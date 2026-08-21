/*
 * axis.c — мосты BTS7960: ШИМ, разгон, запрет одновременного включения плеч.
 *
 * STOP — это выбег (EN в 0), а не оба ШИМ в ноль: при EN = 1 и нулевых входах
 * BTS7960 сажает оба вывода на землю, и привод на 800 Н тормозит обмоткой.
 */

#include "axis.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"

extern TIM_HandleTypeDef htim1;

#define AXIS_PWM_HZ         20000u
#define AXIS_RAMP_MS          200u
#define AXIS_DWELL_MS          50u
#define AXIS_DUTY_MIN_PCT      40u
#define AXIS_HOME_MARGIN_PCT   30u

#define AXIS_STEP_UNKNOWN 0xFFFFu   /* на проводе -1 из README */

#define AXIS_TRIP_CONFIRM_MS  150u
#define AXIS_TRIP_BLIND_MS    (AXIS_DWELL_MS + AXIS_RAMP_MS)  /* 250 мс */
#define ISENSE_ACTIVE_HIGH      1u


typedef enum { DIR_NONE = 0, DIR_EXTEND, DIR_RETRACT } dir_t;

typedef struct {
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;
    uint32_t      ch_extend;        
    uint32_t      ch_retract;
} axis_pins_t;


static const axis_pins_t AXIS_PINS[] = {
    [0] = { AXIS0_EN_GPIO_Port, AXIS0_EN_Pin, TIM_CHANNEL_1, TIM_CHANNEL_2 },
    [1] = { AXIS1_EN_GPIO_Port, AXIS1_EN_Pin, TIM_CHANNEL_3, TIM_CHANNEL_4 },
};

static const struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} ISENSE_PINS[AXIS_COUNT][2] = {
		[0] = {{NULL, 0u}, {NULL, 0u}},
		[1] = { { ISENSE_R_GPIO_Port, ISENSE_R_Pin },
	            { ISENSE_L_GPIO_Port, ISENSE_L_Pin } },
};

typedef struct {
    uint8_t  state;
    uint8_t  fault;
    uint8_t  dir;
    uint16_t step;
    uint16_t from_step;
    uint16_t target;
    uint32_t start_ms;
    uint32_t travel_ms;
    uint8_t  trip_run;
    uint8_t  trip_armed;
    uint32_t trip_since;
} axis_t;

static axis_t   ax[AXIS_COUNT];
static uint32_t duty_full;          /* ARR + 1, то есть 100 % */

/* ------------------------------------------------------------------ железо */

static void axis_apply(unsigned i, dir_t dir, uint32_t duty)
{
    const axis_pins_t *p = &AXIS_PINS[i];

    __HAL_TIM_SET_COMPARE(&htim1, p->ch_extend,  (dir == DIR_EXTEND)  ? duty : 0u);
    __HAL_TIM_SET_COMPARE(&htim1, p->ch_retract, (dir == DIR_RETRACT) ? duty : 0u);

    HAL_GPIO_WritePin(p->en_port, p->en_pin,
                      (dir == DIR_NONE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void axis_publish(unsigned i, uint16_t pos_mm_x10)
{
    const uint16_t base = (uint16_t)(IREG_AXIS0_STATE + 4u * i);

    regfile_set_input((ireg_t)(base + 0u), ax[i].state);
    regfile_set_input((ireg_t)(base + 1u), ax[i].step);
    regfile_set_input((ireg_t)(base + 2u), pos_mm_x10);
    regfile_set_input((ireg_t)(base + 3u), ax[i].fault);
}

static uint32_t ramp_duty(uint32_t run_ms)
{
    const uint32_t min = duty_full * AXIS_DUTY_MIN_PCT / 100u;

    if (run_ms >= AXIS_RAMP_MS) {
        return duty_full;
    }
    return min + ((duty_full - min) * run_ms) / AXIS_RAMP_MS;
}

static uint8_t isense_active(unsigned i, dir_t dir)
{
    if (dir == DIR_NONE) {
        return 0u;
    }

    const unsigned slot = (dir == DIR_EXTEND) ? 0u : 1u;
    GPIO_TypeDef  *port = ISENSE_PINS[i][slot].port;

    if (port == NULL) {
        return 0u;
    }

    const uint8_t high = (HAL_GPIO_ReadPin(port, ISENSE_PINS[i][slot].pin) == GPIO_PIN_SET);
    return ISENSE_ACTIVE_HIGH ? high : (uint8_t)!high;
}

static uint16_t step_to_mm_x10(unsigned i, uint16_t step)
{
    return (uint16_t)((uint32_t)step * AXIS_SPECS[i].step_mm * 10u);
}

static void motion_done(unsigned i)
{
    axis_apply(i, DIR_NONE, 0u);
    ax[i].dir   = DIR_NONE;
    ax[i].step  = ax[i].target;
    ax[i].state = AXIS_STATE_IDLE;
    axis_publish(i, step_to_mm_x10(i, ax[i].step));
}



static void motion_fault(unsigned i, uint8_t fault)
{
    axis_apply(i, DIR_NONE, 0u);
    ax[i].dir   = DIR_NONE;
    ax[i].step  = AXIS_STEP_UNKNOWN;
    ax[i].state = AXIS_STATE_FAULT;
    ax[i].fault = fault;
    axis_publish(i, 0u);
}

static void motion_trip(unsigned i)
{
    if (ax[i].state == AXIS_STATE_HOMING) {
        motion_done(i);
        return;
    }
    motion_fault(i, AXIS_FAULT_STALL);
}



static uint32_t stroke_travel_ms(unsigned i)
{
    return (uint32_t)AXIS_SPECS[i].max_step * AXIS_SPECS[i].step_travel_ms;
}

static uint8_t axis_busy(unsigned i)
{
    return (ax[i].state == AXIS_STATE_MOVING) || (ax[i].state == AXIS_STATE_HOMING);
}






static void motion_abort(unsigned i)
{
    axis_apply(i, DIR_NONE, 0u);
    ax[i].dir = DIR_NONE;

    if (axis_busy(i)) {
        ax[i].step  = AXIS_STEP_UNKNOWN;
        ax[i].state = AXIS_STATE_UNKNOWN;
    }
    axis_publish(i, 0u);
}

static uint16_t motion_start(unsigned i, dir_t dir, uint16_t target,
                             uint32_t travel_ms, uint8_t state)
{
    ax[i].dir       = dir;
    ax[i].from_step = ax[i].step;
    ax[i].target    = target;
    ax[i].travel_ms = travel_ms;
    ax[i].start_ms  = HAL_GetTick();
    ax[i].state     = state;
    ax[i].trip_run   = 0u;
    ax[i].trip_armed = (uint8_t)!isense_active(i, dir);
    axis_apply(i, DIR_NONE, 0u);
    return ACK_RESULT_OK;
}

/* ------------------------------------------------------------------ задачи */

void axis_init(void)
{

    uint32_t timclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0u) {
        timclk *= 2u;
    }
    duty_full = timclk / AXIS_PWM_HZ;
    __HAL_TIM_SET_AUTORELOAD(&htim1, duty_full - 1u);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        ax[i].trip_run   = 0u;
        ax[i].trip_armed = 0u;
        ax[i].trip_since = 0u;
        ax[i].state     = AXIS_STATE_UNKNOWN;
        ax[i].fault     = AXIS_FAULT_NONE;
        ax[i].dir       = DIR_NONE;
        ax[i].step      = AXIS_STEP_UNKNOWN;
        ax[i].from_step = AXIS_STEP_UNKNOWN;
        ax[i].target    = 0u;
        ax[i].travel_ms = 0u;

        axis_apply(i, DIR_NONE, 0u);
        axis_publish(i, 0u);
    }
}

static uint8_t isense_present(unsigned i, dir_t dir)
{
    if (dir == DIR_NONE) {
        return 0u;
    }
    return (uint8_t)(ISENSE_PINS[i][(dir == DIR_EXTEND) ? 0u : 1u].port != NULL);
}

void axis_step(void)
{
    const uint32_t now = HAL_GetTick();

    for (unsigned i = 0; i < AXIS_COUNT; i++) {
        if (!axis_busy(i)) {
            continue;
        }

        const uint32_t elapsed = now - ax[i].start_ms;

        if (elapsed < AXIS_DWELL_MS) {
            axis_apply(i, DIR_NONE, 0u);
            continue;
        }

        const uint32_t run = elapsed - AXIS_DWELL_MS;

        if (run >= ax[i].travel_ms) {
            if ((ax[i].state == AXIS_STATE_HOMING) && ax[i].trip_armed
                && isense_present(i, (dir_t)ax[i].dir)) {
                motion_fault(i, AXIS_FAULT_TIMEOUT);   /* ждали упор, не дождались */
            } else {
                motion_done(i);
            }
            continue;
        }

        axis_apply(i, (dir_t)ax[i].dir, ramp_duty(run));
        if (ax[i].trip_armed && (elapsed >= AXIS_TRIP_BLIND_MS)) {
            if (!isense_active(i, (dir_t)ax[i].dir)) {
                ax[i].trip_run = 0u;
            } else if (!ax[i].trip_run) {
                ax[i].trip_run   = 1u;
                ax[i].trip_since = now;
            } else if ((now - ax[i].trip_since) >= AXIS_TRIP_CONFIRM_MS) {
                motion_trip(i);
                continue;
            }
        }
        uint16_t pos = 0u;
        if (ax[i].state == AXIS_STATE_MOVING) {
            const int32_t from = step_to_mm_x10(i, ax[i].from_step);
            const int32_t to   = step_to_mm_x10(i, ax[i].target);
            pos = (uint16_t)(from + ((to - from) * (int32_t)run) / (int32_t)ax[i].travel_ms);
        }
        axis_publish(i, pos);
    }
}

void axis_failsafe_step(void)
{
    if (uart_link_is_up()) {
        return;
    }
    for (unsigned i = 0; i < AXIS_COUNT; i++) {
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
    if (ax[i].step == AXIS_STEP_UNKNOWN) {
        return ACK_RESULT_REFUSED_SAFETY;   
    }
    if (step == ax[i].step) {
        return ACK_RESULT_OK;
    }

    const dir_t    dir   = (step > ax[i].step) ? DIR_EXTEND : DIR_RETRACT;
    const uint16_t delta = (dir == DIR_EXTEND) ? (uint16_t)(step - ax[i].step)
                                               : (uint16_t)(ax[i].step - step);

    return motion_start(i, dir, step,
                        (uint32_t)delta * AXIS_SPECS[i].step_travel_ms,
                        AXIS_STATE_MOVING);
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
    return motion_start(i, DIR_RETRACT, 0u, travel, AXIS_STATE_HOMING);
}
