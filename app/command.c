#include "command.h"
#include "release.h"
#include "axis.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"



static uint16_t last_seq;

static uint16_t do_release(uint16_t unit, uint16_t reserved)
{
    if (reserved != 0u) {
        return ACK_RESULT_BAD_ARG;
    }

    return release_cmd_start(unit);
}

static uint16_t do_axis(uint16_t code, uint16_t axis, uint16_t step)
{
    if (axis >= AXIS_COUNT) {
        return ACK_RESULT_BAD_ARG;
    }
    if (code == CMD_CODE_AXIS_GOTO && step > AXIS_SPECS[axis].max_step) {
        return ACK_RESULT_BAD_ARG;
    }

    if (code != CMD_CODE_AXIS_STOP && !uart_link_is_up()) {
        return ACK_RESULT_REFUSED_LINK_LOST;
    }

    switch (code) {
        case CMD_CODE_AXIS_GOTO: return axis_cmd_goto(axis, step);
        case CMD_CODE_AXIS_STOP: return axis_cmd_stop(axis);
        default:                 return step == 1 ? axis_cmd_calibrate(axis) :axis_cmd_home(axis);
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

