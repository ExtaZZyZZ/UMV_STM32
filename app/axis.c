

#include "axis.h"
#include "axis_odom.h"
#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"



#define AXIS_DWELL_MS          50u
#define AXIS_SPINUP_MS        300u
#define AXIS_ARRIVE_MS        100u
#define AXIS_HOME_MARGIN_PCT   30u
#define AXIS_ENDSTOP_PUSH_MS 2000u
#define ISENSE_ACTIVE_HIGH      1u

#define AXIS_STEP_UNKNOWN 0xFFFFu
#define AXIS_TIMEOUT_MARGIN_PCT 50u
#define SONAR_MAST_AXIS 1u

_Static_assert(SONAR_MAST_AXIS < AXIS_COUNT, "SONAR_MAST_AXIS out of range");

#define AXIS_USE_ISENSE 0u


typedef enum { DIR_NONE = 0, DIR_EXTEND, DIR_RETRACT } dir_t;
typedef enum { REF_NONE = 0, REF_LOW, REF_HIGH } ref_t;
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


static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} ISENSE_PINS[AXIS_COUNT][2] = {
    [0] = { { NULL, 0u }, { NULL, 0u } },
    [1] = { { ISENSE_R_GPIO_Port, ISENSE_R_Pin },
            { ISENSE_L_GPIO_Port, ISENSE_L_Pin } },
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
    uint8_t  cur_valid;
    uint8_t  cur_seen;
    uint8_t  cur_off_run;
    uint32_t cur_off_since;
    uint8_t  boot_home;
} axis_t;


static axis_t  ax[AXIS_COUNT];
static uint8_t boot_pending = 1u;



static void axis_apply(unsigned i, dir_t dir)
{
    const axis_pins_t *p = &AXIS_PINS[i];


    HAL_GPIO_WritePin(p->ext_port, p->ext_pin,
                      (dir == DIR_EXTEND)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->ret_port, p->ret_pin,
                      (dir == DIR_RETRACT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->en_port, p->en_pin,
                      (dir == DIR_NONE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t isense_present(unsigned i, dir_t dir)
{
    if (dir == DIR_NONE) {
        return 0u;
    }
    return (uint8_t)(ISENSE_PINS[i][(dir == DIR_EXTEND) ? 0u : 1u].port != NULL);
}

static uint8_t isense_active(unsigned i, dir_t dir)
{
    if (!isense_present(i, dir)) {
        return 0u;
    }

    const unsigned slot = (dir == DIR_EXTEND) ? 0u : 1u;
    const uint8_t  high = (HAL_GPIO_ReadPin(ISENSE_PINS[i][slot].port,
                                            ISENSE_PINS[i][slot].pin) == GPIO_PIN_SET);
    return ISENSE_ACTIVE_HIGH ? high : (uint8_t)!high;
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

static uint8_t use_current(unsigned i)
{
    return (uint8_t)(AXIS_USE_ISENSE
                     && ax[i].cur_valid
                     && isense_present(i, (dir_t)ax[i].dir)
                     && target_is_endstop(i));
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

    if (ax[i].endstop == REF_LOW) {
        ax[i].pos_x10 = 0;
    } else if (ax[i].endstop == REF_HIGH) {
        ax[i].pos_x10 = axis_max_x10(i);
    } else {
        ax[i].pos_x10 = odom_step_to_x10(ax[i].target, AXIS_SPECS[i].step_mm);
    }

    ax[i].ref       = ax[i].endstop;
    ax[i].dir       = DIR_NONE;
    ax[i].known     = 1u;
    ax[i].state     = AXIS_STATE_IDLE;
    ax[i].boot_home = 0u;
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

    ax[i].cur_valid     = (uint8_t)!isense_active(i, dir);
    ax[i].cur_seen      = 0u;
    ax[i].cur_off_run   = 0u;
    ax[i].cur_off_since = 0u;
    if (use_current(i)) {
        ax[i].travel_ms = (stroke_travel_ms(i) * (100u + AXIS_TIMEOUT_MARGIN_PCT)) / 100u;
    }
    axis_apply(i, DIR_NONE);
    return ACK_RESULT_OK;
}

static uint8_t arrived_by_current(unsigned i, uint32_t elapsed, uint32_t now)
{
    const uint8_t on = isense_active(i, (dir_t)ax[i].dir);

    if (elapsed < (AXIS_DWELL_MS + AXIS_SPINUP_MS)) {
        if (on) {
            ax[i].cur_seen = 1u;
        }
        return 0u;
    }

    if (!ax[i].cur_seen) {
        return 1u;
    }

    if (on) {
        ax[i].cur_off_run = 0u;
        return 0u;
    }

    if (!ax[i].cur_off_run) {
        ax[i].cur_off_run   = 1u;
        ax[i].cur_off_since = now;
        return 0u;
    }

    return (uint8_t)((now - ax[i].cur_off_since) >= AXIS_ARRIVE_MS);
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
        ax[i].cur_valid     = 0u;
        ax[i].cur_seen      = 0u;
        ax[i].cur_off_run   = 0u;
        ax[i].cur_off_since = 0u;
        ax[i].boot_home     = 0u;

        axis_apply(i, DIR_NONE);
        axis_publish(i);
    }
}

void axis_step(void)
{
    /* Хоминг при старте: здесь, а не в axis_init() — там ещё не крутится
     * планировщик, и выдержка утекла бы на печати самопроверки. */
    if (boot_pending) {
        boot_pending = 0u;
        for (unsigned i = 0; i < AXIS_COUNT; i++) {
            if (axis_cmd_home(i) == ACK_RESULT_OK) {
                ax[i].boot_home = 1u;
            }
        }
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
            if (use_current(i)) {
                motion_fault(i, AXIS_FAULT_TIMEOUT);
            } else {
                motion_done(i);
            }
            continue;
        }

        axis_apply(i, (dir_t)ax[i].dir);

        if (use_current(i) && arrived_by_current(i, elapsed, now)) {
            motion_done(i);
            continue;
        }

        ax[i].pos_x10 = reckon_x10(i, now);
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
            continue;            /* BOOT_HOME: единственное исключение */
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
