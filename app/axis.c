

#include "axis.h"
#include "axis_odom.h"
#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"
#include "encoder.h"

#define AXIS_DWELL_MS          50u
#define AXIS_HOME_MARGIN_PCT   30u
#define AXIS_ENDSTOP_PUSH_MS 2000u

#define AXIS_STEP_UNKNOWN 0xFFFFu
#define AXIS_TIMEOUT_MARGIN_PCT 50u
#define SONAR_MAST_AXIS 1u

#define ENC_COUNTS_LOW  900
#define ENC_COUNTS_HIGH 132
#define AXIS_STALL_COUNTS 2
#define AXIS_STALL_MS 2000u
#define CALIB_MIN_SPAN 20

#define CAL1     0x43414C31
#define ADDRESS_FLASH 0x0807F800u

typedef struct {
    uint32_t mark;
    int32_t lower_stop;
    int32_t upper_stop;
    uint32_t additional;
} flash_calib_t;
_Static_assert(sizeof(flash_calib_t) % 8 == 0, "The structure is not a multiple of 8.");
_Static_assert(SONAR_MAST_AXIS < AXIS_COUNT, "SONAR_MAST_AXIS out of range");
static int32_t calib_first_val = ENC_COUNTS_LOW;
static int32_t calib_second_val = ENC_COUNTS_HIGH;
static int32_t calib_temp_val = 0;

typedef enum { CALIB_OFF, CALIB_EXTEND, CALIB_RETRACT} calib_t;
typedef enum { DIR_NONE = 0, DIR_EXTEND, DIR_RETRACT } dir_t;
typedef enum { REF_NONE = 0, REF_LOW, REF_HIGH } ref_t;
static calib_t step_calib = CALIB_OFF;

typedef struct {
    GPIO_TypeDef *en_port;   uint16_t en_pin;
    GPIO_TypeDef *ext_port;  uint16_t ext_pin;   /* RPWM */
    GPIO_TypeDef *ret_port;  uint16_t ret_pin;   /* LPWM */
} axis_pins_t;

static const axis_pins_t AXIS_PINS[AXIS_COUNT] = {
    [0] = { AXIS0_EN_GPIO_Port,   AXIS0_EN_Pin,
            AXIS0_RPWM_GPIO_Port, AXIS0_RPWM_Pin,
            AXIS0_LPWM_GPIO_Port, AXIS0_LPWM_Pin },
    [1] = { AXIS1_EN_GPIO_Port,   AXIS1_EN_Pin,
            AXIS1_RPWM_GPIO_Port, AXIS1_RPWM_Pin,
            AXIS1_LPWM_GPIO_Port, AXIS1_LPWM_Pin },
};

typedef struct {
    uint8_t  state;
    uint8_t  fault;
    uint8_t  dir;
    uint8_t  known;
    uint8_t  ref;
    uint8_t  endstop;
    int32_t  pos_x10;
    int32_t  origin_x10;
    uint16_t target;
    uint32_t start_ms;
    uint32_t travel_ms;
    uint8_t  boot_home;
    int32_t stall_count;
	uint32_t stall_since;
} axis_t;

static uint16_t motion_start(unsigned i, dir_t dir, uint16_t target,
                             uint32_t travel_ms, uint8_t state, ref_t endstop);
static void motion_fault(unsigned i, uint8_t fault);
static axis_t  ax[AXIS_COUNT];
static uint8_t boot_pending = 1u;

static HAL_StatusTypeDef flash_write(int32_t low, int32_t high)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef flash;
    flash.TypeErase = FLASH_TYPEERASE_PAGES;
    flash.Banks = FLASH_BANK_2;
    flash.Page = 127u;
    flash.NbPages = 1u;
    uint32_t page_error;
    if(HAL_FLASHEx_Erase(&flash, &page_error) != HAL_OK) 
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    uint64_t data = ((uint64_t)(uint32_t)low << 32) | (uint64_t)CAL1;
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                                 ADDRESS_FLASH, data);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    data   = (uint64_t)(uint32_t)high;
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               ADDRESS_FLASH + 8u, data);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    HAL_FLASH_Lock();
    return status;

}

static uint8_t flash_read(void)
{
    const flash_calib_t* rec = (const flash_calib_t *)ADDRESS_FLASH;
    if(rec->mark != CAL1) return 0u;
    calib_first_val = rec->lower_stop;
    calib_second_val = rec->upper_stop;
    return 1u;
}
static void axis_apply(unsigned i, dir_t dir)
{
    const axis_pins_t *p = &AXIS_PINS[i];


    HAL_GPIO_WritePin(p->ext_port, p->ext_pin,
                      (dir != DIR_NONE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->ret_port, p->ret_pin,
                      (dir == DIR_RETRACT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->en_port, p->en_pin,
                      (dir == DIR_NONE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static int32_t axis_max_x10(unsigned i)
{
    return odom_step_to_x10(AXIS_SPECS[i].max_step, AXIS_SPECS[i].step_mm);
}

static uint16_t axis_step_now(unsigned i)
{
    return ax[i].known
         ? odom_x10_to_step(ax[i].pos_x10, AXIS_SPECS[i].step_mm)
         : AXIS_STEP_UNKNOWN;
}


static uint32_t stroke_travel_ms(unsigned i)
{
    return (uint32_t)AXIS_SPECS[i].max_step * AXIS_SPECS[i].step_travel_ms;
}

static uint8_t axis_busy(unsigned i)
{
    return (ax[i].state == AXIS_STATE_MOVING) || (ax[i].state == AXIS_STATE_HOMING);
}


static uint8_t target_is_endstop(unsigned i)
{
    return (uint8_t)((ax[i].target == 0u)
                     || (ax[i].target == AXIS_SPECS[i].max_step));
}

static void axis_publish(unsigned i)
{
    const uint16_t base = (uint16_t)(IREG_AXIS0_STATE + 4u * i);

    regfile_set_input((ireg_t)(base + 0u), ax[i].state);
    regfile_set_input((ireg_t)(base + 1u), axis_step_now(i));
    regfile_set_input((ireg_t)(base + 2u), ax[i].known ? (uint16_t)ax[i].pos_x10 : 0u);
    regfile_set_input((ireg_t)(base + 3u), ax[i].fault);
}

static int32_t reckon_x10(unsigned i, uint32_t now)
{
    if (ax[i].dir == DIR_NONE) {
        return ax[i].pos_x10;
    }

    const uint32_t elapsed = now - ax[i].start_ms;
    const uint32_t run     = (elapsed > AXIS_DWELL_MS) ? (elapsed - AXIS_DWELL_MS) : 0u;
    const int32_t  gone    = odom_x10_from_ms(run, AXIS_SPECS[i].step_travel_ms,
                                              AXIS_SPECS[i].step_mm);
    const int32_t  max     = axis_max_x10(i);

    int32_t pos = (ax[i].dir == DIR_EXTEND) ? (ax[i].origin_x10 + gone)
                                            : (ax[i].origin_x10 - gone);

    if (pos < 0)   { pos = 0;   }
    if (pos > max) { pos = max; }
    return pos;
}

static void motion_done(unsigned i)
{
    axis_apply(i, DIR_NONE);
    if(!AXIS_SPECS[i].has_encoder) {
        if (ax[i].endstop == REF_LOW) {
            ax[i].pos_x10 = 0;
        } else if (ax[i].endstop == REF_HIGH) {
            ax[i].pos_x10 = axis_max_x10(i);
        } else {
            ax[i].pos_x10 = odom_step_to_x10(ax[i].target, AXIS_SPECS[i].step_mm);
        }
    }

    ax[i].ref       = ax[i].endstop;
    ax[i].dir       = DIR_NONE;
    ax[i].known     = 1u;
    ax[i].state     = AXIS_STATE_IDLE;
    ax[i].boot_home = 0u;
    if(AXIS_SPECS[i].has_encoder) {
        if (step_calib == CALIB_RETRACT) {
            calib_temp_val = encoder_counts();
            step_calib = CALIB_EXTEND;
            motion_start(i, DIR_EXTEND, AXIS_SPECS[i].max_step, 0u, AXIS_STATE_MOVING, REF_HIGH);
            return;
        }
        else if (step_calib == CALIB_EXTEND) {
            step_calib = CALIB_OFF;
            const int32_t high = encoder_counts();                    
            const int32_t span = odom_turn_diff(calib_temp_val, high);
            if ((span < CALIB_MIN_SPAN) && (-span < CALIB_MIN_SPAN)) {
                motion_fault(i, AXIS_FAULT_STALL);
                return;
            }
            calib_first_val = calib_temp_val;
            calib_second_val = high;
            flash_write(calib_first_val, calib_second_val);
        }
}
    axis_publish(i);
    
}

static void motion_abort(unsigned i)
{
    const int32_t pos = reckon_x10(i, HAL_GetTick());

    axis_apply(i, DIR_NONE);

    if (axis_busy(i)) {
        ax[i].pos_x10 = pos;
        ax[i].state   = ax[i].known ? AXIS_STATE_IDLE : AXIS_STATE_UNKNOWN;
        ax[i].ref     = REF_NONE;
        ax[i].endstop = REF_NONE;
    }
    if(AXIS_SPECS[i].has_encoder) step_calib = CALIB_OFF;
    
    ax[i].dir       = DIR_NONE;
    ax[i].boot_home = 0u;
    axis_publish(i);
}

static void motion_fault(unsigned i, uint8_t fault)
{
    axis_apply(i, DIR_NONE);
    ax[i].dir       = DIR_NONE;
    ax[i].known     = 0u;
    ax[i].ref       = REF_NONE;
    ax[i].endstop   = REF_NONE;
    ax[i].state     = AXIS_STATE_FAULT;
    ax[i].fault     = fault;
    ax[i].boot_home = 0u;
    if(AXIS_SPECS[i].has_encoder) step_calib = CALIB_OFF;
    axis_publish(i);
}

static uint16_t motion_start(unsigned i, dir_t dir, uint16_t target,
                             uint32_t travel_ms, uint8_t state, ref_t endstop)
{
    ax[i].dir        = dir;
    ax[i].origin_x10 = ax[i].pos_x10;
    ax[i].target     = target;
    ax[i].endstop    = (uint8_t)endstop;
    ax[i].ref        = REF_NONE;
    ax[i].travel_ms  = travel_ms;
    ax[i].start_ms   = HAL_GetTick();
    ax[i].state      = state;

    ax[i].stall_count = encoder_counts();
    ax[i].stall_since = ax[i].start_ms;
    if (AXIS_SPECS[i].has_encoder) {
        ax[i].travel_ms = (stroke_travel_ms(i) * (100u + AXIS_TIMEOUT_MARGIN_PCT)) / 100u;
    }
    axis_apply(i, DIR_NONE);
    return ACK_RESULT_OK;
}
static uint8_t crossed_target(unsigned i)
{
    const int32_t t = odom_step_to_x10(ax[i].target, AXIS_SPECS[i].step_mm);
    return (ax[i].dir == DIR_EXTEND) ? (uint8_t)(ax[i].pos_x10 >= t)
                                        :(uint8_t)(ax[i].pos_x10 <=t);
}
static uint8_t stalled(unsigned i, uint32_t now)
{
	const int32_t c = encoder_counts();
	const int32_t d = odom_turn_diff(ax[i].stall_count, c);
	if ((d >= AXIS_STALL_COUNTS) || (-d >= AXIS_STALL_COUNTS )) {
        ax[i].stall_count = c;
        ax[i].stall_since = now;
        return 0u;
    }
    return (uint8_t)((now - ax[i].stall_since) >= AXIS_STALL_MS);
}



void axis_init(void)
{
    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        ax[i].state         = AXIS_STATE_UNKNOWN;
        ax[i].fault         = AXIS_FAULT_NONE;
        ax[i].dir           = DIR_NONE;
        ax[i].known         = 0u;
        ax[i].ref           = REF_NONE;
        ax[i].endstop       = REF_NONE;
        ax[i].pos_x10       = 0;
        ax[i].origin_x10    = 0;
        ax[i].target        = 0u;
        ax[i].start_ms      = 0u;
        ax[i].travel_ms     = 0u;
        ax[i].boot_home     = 0u;

        axis_apply(i, DIR_NONE);
        axis_publish(i);
    }
    (void)flash_read();

}

void axis_step(void)
{
	for (unsigned i = 0; i < AXIS_COUNT; i++) {
		if (AXIS_SPECS[i].has_encoder && encoder_is_up()) {
			ax[i].pos_x10 = odom_x10_from_counts(encoder_counts(), calib_first_val, calib_second_val, axis_max_x10(i));
			ax[i].known = 1u;
		
        if (ax[i].state == AXIS_STATE_UNKNOWN) {
            ax[i].state = AXIS_STATE_IDLE;
            }
        }
	}

    if (boot_pending) {
        for(unsigned i = 0; i < AXIS_COUNT; i++) {
            if (AXIS_SPECS[i].has_encoder) {
                continue;
            }
            if (axis_cmd_home(i) == ACK_RESULT_OK) {
                ax[i].boot_home = 1u;
            }
        }
        boot_pending = 0u;
    }


    const uint32_t now = HAL_GetTick();

    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        if (!axis_busy(i)) {
            continue;
        }

        const uint32_t elapsed = now - ax[i].start_ms;

        if (elapsed < AXIS_DWELL_MS) {
            axis_apply(i, DIR_NONE);
            continue;
        }

        const uint32_t run = elapsed - AXIS_DWELL_MS;

        if (run >= ax[i].travel_ms) {
            if (AXIS_SPECS[i].has_encoder) {
                motion_fault(i, AXIS_FAULT_TIMEOUT);
            } else {
                motion_done(i);
            }
            continue;
        }

        axis_apply(i, (dir_t)ax[i].dir);
        if (AXIS_SPECS[i].has_encoder) {
            if(!encoder_is_up()) {
                motion_fault(i, AXIS_FAULT_ENCODER_LOST);
                continue;
            }
            if (stalled(i, now)) {
                if (target_is_endstop(i)) {
                motion_done(i);               
            }
             else {
                motion_fault(i, AXIS_FAULT_STALL);
            }
            continue;
        }
            if (!target_is_endstop(i) && crossed_target(i)) {
                motion_done(i);
                continue;
            }
            
        }
        if (!AXIS_SPECS[i].has_encoder) {
            ax[i].pos_x10 = reckon_x10(i, now);
        }
    }

    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        axis_publish(i);
    }
    HAL_GPIO_WritePin(TURN_ON_SONAR_GPIO_Port, TURN_ON_SONAR_Pin,
                      (ax[SONAR_MAST_AXIS].ref == REF_HIGH) ? GPIO_PIN_SET
                                                            : GPIO_PIN_RESET);
    
}


void axis_failsafe_step(void)
{
    if (uart_link_is_up()) {
        return;
    }
    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        if (ax[i].boot_home) {
            continue;
        }
        if (axis_busy(i)) {
            motion_abort(i);
        }
    }
}

uint16_t axis_cmd_goto(uint16_t i, uint16_t step)
{
    if (axis_busy(i)) {
        return ACK_RESULT_REFUSED_BUSY;
    }
    if (ax[i].state == AXIS_STATE_FAULT) {
        return ACK_RESULT_REFUSED_SAFETY;
    }


    /* Ноль — упор, а не координата: ход туда возвращает привязку, поэтому идёт
     * с запасом хоминга. Состояние при этом MOVING — HOMING в карте значит
     * «положение неизвестно», а здесь оно известно. */
    if (step == 0u) {
        if (ax[i].ref == REF_LOW) {
            return ACK_RESULT_OK;
        }
        return motion_start(i, DIR_RETRACT, 0u,
                            (stroke_travel_ms(i) * (100u + AXIS_HOME_MARGIN_PCT)) / 100u,
                            AXIS_STATE_MOVING, REF_LOW);
    }

    if (!ax[i].known) {
        return ACK_RESULT_REFUSED_SAFETY;
    }

    const uint8_t top   = (uint8_t)(step == AXIS_SPECS[i].max_step);
    const int32_t delta = odom_step_to_x10(step, AXIS_SPECS[i].step_mm) - ax[i].pos_x10;

    if (top ? (ax[i].ref == REF_HIGH) : (axis_step_now(i) == step)) {
        return ACK_RESULT_OK;
    }

    const dir_t dir = (delta >= 0) ? DIR_EXTEND : DIR_RETRACT;

    uint32_t travel = odom_ms_from_x10((delta >= 0) ? delta : -delta,
                                       AXIS_SPECS[i].step_travel_ms,
                                       AXIS_SPECS[i].step_mm);
    if (top) {
        travel += AXIS_ENDSTOP_PUSH_MS;
    }

    return motion_start(i, dir, step, travel, AXIS_STATE_MOVING,
                        top ? REF_HIGH : REF_NONE);
}


uint16_t axis_cmd_stop(uint16_t i)
{
    motion_abort(i);
    return ACK_RESULT_OK;
}

uint16_t axis_cmd_home(uint16_t i)
{
    if (axis_busy(i)) {
        return ACK_RESULT_REFUSED_BUSY;
    }

    const uint32_t travel = (stroke_travel_ms(i) * (100u + AXIS_HOME_MARGIN_PCT)) / 100u;

    ax[i].fault = AXIS_FAULT_NONE;
    return motion_start(i, DIR_RETRACT, 0u, travel, AXIS_STATE_HOMING, REF_LOW);
}

bool axis_calibration_active(void)
{
    return step_calib != CALIB_OFF;
}

uint16_t axis_cmd_calibrate(uint16_t i)
{
    if (!AXIS_SPECS[i].has_encoder) {
        return ACK_RESULT_BAD_ARG;
    }
    if (axis_busy(i)) {
        return ACK_RESULT_REFUSED_BUSY;
    }
    if (!encoder_is_up()) {
        return ACK_RESULT_REFUSED_SAFETY;
    }

    ax[i].fault = AXIS_FAULT_NONE;
    step_calib  = CALIB_RETRACT;
    return motion_start(i, DIR_RETRACT, 0u, 0u, AXIS_STATE_HOMING, REF_LOW);
}

