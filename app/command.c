

#include "command.h"

#include "main.h"
#include "axis.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"



static uint16_t last_seq;

static struct {
    uint8_t  active;
    uint32_t started_ms;
    uint32_t duration_ms;
} pulses[RELEASE_COUNT];

static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       active_high;
}
 LOCK_PINS[] = {
    [0] = { LOCK1_GPIO_Port, LOCK1_Pin, 1u },
    [1] = { LOCK2_GPIO_Port, LOCK2_Pin, 1u },
};
static void release_coil_set(uint8_t unit, uint8_t on)
 {
     const GPIO_PinState level =
         (on == LOCK_PINS[unit].active_high) ? GPIO_PIN_SET : GPIO_PIN_RESET;
     HAL_GPIO_WritePin(LOCK_PINS[unit].port, LOCK_PINS[unit].pin, level);
 }

static uint16_t do_release(uint16_t unit, uint16_t duration_ms)
{
    if (unit >= RELEASE_COUNT) {
        return ACK_RESULT_BAD_ARG;
    }
    if (duration_ms == 0u || duration_ms > RELEASE_SPECS[unit].pulse_ms_max) {
        return ACK_RESULT_BAD_ARG;
    }


    pulses[unit].started_ms  = HAL_GetTick();
    pulses[unit].duration_ms = duration_ms;
    pulses[unit].active      = 1u;
    release_coil_set((uint8_t)unit, 1u);

    return ACK_RESULT_OK;
}

static uint16_t do_axis(uint16_t code, uint16_t axis, uint16_t step)
{
    if (axis >= AXIS_COUNT) {
        return ACK_RESULT_BAD_ARG;
    }
    if (code == CMD_CODE_AXIS_GOTO && step > AXIS_SPECS[axis].max_step) {
        return ACK_RESULT_BAD_ARG;
    }

    /* STOP пропускаем всегда: отказать в остановке хуже, чем исполнить её
     * на подозрительной линии. */
    if (code != CMD_CODE_AXIS_STOP && !uart_link_is_up()) {
        return ACK_RESULT_REFUSED_LINK_LOST;
    }

    switch (code) {
        case CMD_CODE_AXIS_GOTO: return axis_cmd_goto(axis, step);
        case CMD_CODE_AXIS_STOP: return axis_cmd_stop(axis);
        default:                 return axis_cmd_home(axis);
    }
}

static uint16_t command_execute(uint16_t code, uint16_t arg0, uint16_t arg1)
{
    switch (code) {
        case CMD_CODE_NOP:
            return ACK_RESULT_OK;

        case CMD_CODE_RELEASE:

            return do_release(arg0, arg1);

        case CMD_CODE_AXIS_GOTO:
        case CMD_CODE_AXIS_STOP:
        case CMD_CODE_AXIS_HOME:
            return do_axis(code, arg0, arg1);

        default:
            return ACK_RESULT_UNKNOWN_CMD;
    }
}

void command_init(void)
{
    last_seq = 0u;
    for (unsigned i = 0; i < RELEASE_COUNT; i++) {
        pulses[i].active      = 0u;
        pulses[i].started_ms  = 0u;
        pulses[i].duration_ms = 0u;
        release_coil_set((uint8_t)i, 0u);
    }
}

void command_step(void)
{
    const uint16_t seq = regfile_get_holding(HREG_CMD_SEQ);
    if (seq == 0u) {
        return;              
    }
    if (seq == last_seq) {
        return;             
    }
    const uint16_t result = command_execute(regfile_get_holding(HREG_CMD_CODE),
                                            regfile_get_holding(HREG_CMD_ARG0),
                                            regfile_get_holding(HREG_CMD_ARG1));


    last_seq = seq;
    regfile_set_input(IREG_ACK_RESULT, result);
    regfile_set_input(IREG_ACK_SEQ,    seq);
}

void command_pulse_step(void)
{
    const uint32_t now = HAL_GetTick();

    for (unsigned i = 0; i < RELEASE_COUNT; i++) {
        if (!pulses[i].active) {
            continue;
        }
        if ((now - pulses[i].started_ms) >= pulses[i].duration_ms) {
            release_coil_set((uint8_t)i, 0u);
            pulses[i].active = 0u;
        }
    }
}

uint8_t command_release_active(uint8_t unit)
{
    return (unit < RELEASE_COUNT) ? pulses[unit].active : 0u;
}
