

#include "axis.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"

#define AXIS_DWELL_MS          50u
#define AXIS_SPINUP_MS        300u
#define AXIS_ARRIVE_MS        100u
#define AXIS_HOME_MARGIN_PCT   30u
#define ISENSE_ACTIVE_HIGH      1u

#define AXIS_STEP_UNKNOWN 0xFFFFu
#define AXIS_TIMEOUT_MARGIN_PCT 50u

/* 0 — на датчик тока не смотрим, ход только по времени. Версия с включённым
 * детектом лежит в коммите eab0601, вернуть когда вход IS будет исправен. */
#define AXIS_USE_ISENSE 0u

#define AXIS_BOOT_HOME 1u

typedef enum { DIR_NONE = 0, DIR_EXTEND, DIR_RETRACT } dir_t;

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
    uint16_t step;
    uint16_t from_step;
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



static uint16_t step_to_mm_x10(unsigned i, uint16_t step)
{
    return (uint16_t)((uint32_t)step * AXIS_SPECS[i].step_mm * 10u);
}

static uint32_t stroke_travel_ms(unsigned i)
{
    return (uint32_t)AXIS_SPECS[i].max_step * AXIS_SPECS[i].step_travel_ms;
}

static uint8_t axis_busy(unsigned i)
{
    return (ax[i].state == AXIS_STATE_MOVING) || (ax[i].state == AXIS_STATE_HOMING);
}

/* Пропадание тока означает прибытие только если целимся в упор. У оси 0 сто
 * пятьдесят промежуточных положений — там это неверно. */
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

static void axis_publish(unsigned i, uint16_t pos_mm_x10)
{
    const uint16_t base = (uint16_t)(IREG_AXIS0_STATE + 4u * i);

    regfile_set_input((ireg_t)(base + 0u), ax[i].state);
    regfile_set_input((ireg_t)(base + 1u), ax[i].step);
    regfile_set_input((ireg_t)(base + 2u), pos_mm_x10);
    regfile_set_input((ireg_t)(base + 3u), ax[i].fault);
}

static void motion_done(unsigned i)
{
    axis_apply(i, DIR_NONE);
    ax[i].dir       = DIR_NONE;
    ax[i].step      = ax[i].target;
    ax[i].state     = AXIS_STATE_IDLE;
    ax[i].boot_home = 0u;                    /* BOOT_HOME */
    axis_publish(i, step_to_mm_x10(i, ax[i].step));
}

static void motion_abort(unsigned i)
{
    axis_apply(i, DIR_NONE);
    ax[i].dir       = DIR_NONE;
    ax[i].boot_home = 0u;                    /* BOOT_HOME */

    if (axis_busy(i)) {
        ax[i].step  = AXIS_STEP_UNKNOWN;
        ax[i].state = AXIS_STATE_UNKNOWN;
    }
    axis_publish(i, 0u);
}

static void motion_fault(unsigned i, uint8_t fault)
{
    axis_apply(i, DIR_NONE);
    ax[i].dir       = DIR_NONE;
    ax[i].step      = AXIS_STEP_UNKNOWN;
    ax[i].state     = AXIS_STATE_FAULT;
    ax[i].fault     = fault;
    ax[i].boot_home = 0u;                    /* BOOT_HOME */
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
        ax[i].step          = AXIS_STEP_UNKNOWN;
        ax[i].from_step     = AXIS_STEP_UNKNOWN;
        ax[i].target        = 0u;
        ax[i].travel_ms     = 0u;
        ax[i].cur_valid     = 0u;
        ax[i].cur_seen      = 0u;
        ax[i].cur_off_run   = 0u;
        ax[i].cur_off_since = 0u;
        ax[i].boot_home     = 0u;

        axis_apply(i, DIR_NONE);
        axis_publish(i, 0u);
    }
}

void axis_step(void)
{
    const uint32_t now = HAL_GetTick();

    /* BOOT_HOME: здесь, а не в axis_init() — там ещё не крутится планировщик,
     * и выдержка утекла бы на печати самопроверки. */
    if (boot_pending) {
        boot_pending = 0u;
        if (axis_cmd_home(AXIS_BOOT_HOME) == ACK_RESULT_OK) {
            ax[AXIS_BOOT_HOME].boot_home = 1u;
        }
    }

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
            /* Ток должен был пропасть раньше: заклинило или питание просело.
             * Без входа таймер остаётся штатным финишем. */
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
