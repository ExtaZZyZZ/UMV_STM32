#include "host_hal.h"
#include "test_support.h"

#include "command.h"
#include "regfile.h"
#include "uart_link.h"

static uint16_t dispatch(uint16_t seq, uint16_t code, uint16_t arg0, uint16_t arg1)
{
    (void)regfile_write_holding(HREG_CMD_SEQ, seq);
    (void)regfile_write_holding(HREG_CMD_CODE, code);
    (void)regfile_write_holding(HREG_CMD_ARG0, arg0);
    (void)regfile_write_holding(HREG_CMD_ARG1, arg1);
    command_step();

    uint16_t value = 0u;
    (void)regfile_read_input(IREG_ACK_SEQ, &value);
    CHECK(value == seq);
    (void)regfile_read_input(IREG_ACK_RESULT, &value);
    return value;
}

void test_command_run(void)
{
    host_hal_reset();
    regfile_init();
    command_init();

    CHECK(dispatch(1u, CMD_CODE_AXIS_GOTO, AXIS_COUNT, 0u) == ACK_RESULT_BAD_ARG);
    CHECK(host_hal_axis_command_calls() == 0u);

    CHECK(uart_link_is_up() == 0u);
    CHECK(dispatch(2u, CMD_CODE_AXIS_STOP, 0u, 0u) == ACK_RESULT_OK);
    CHECK(host_hal_axis_command_calls() == 1u);
}
