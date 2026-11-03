#include "host_hal.h"
#include "test_support.h"

#include "inputs.h"
#include "regfile.h"
#include "release.h"
#include "selftest.h"

#include <stdio.h>
#include <string.h>

static void print_selftest_result(void)
{
    const char *line = host_hal_uart_log();
    while (*line != '\0') {
        const char *end = strchr(line, '\n');
        size_t length = end != NULL ? (size_t)(end - line) : strlen(line);
        if (length != 0u && line[length - 1u] == '\r') length--;
        if (strncmp(line, "FAIL ", 5u) == 0 || strncmp(line, "selftest: ", 10u) == 0) {
            printf("%.*s\n", (int)length, line);
        }
        if (end == NULL) break;
        line = end + 1;
    }
}

static void boot_case(GPIO_PinState presence_level)
{
    host_hal_reset();
    /* Drive confirmed contacts independently of the production GPIO aliases. */
    host_hal_set_input(GPIOD, GPIO_PIN_1, presence_level);
    host_hal_set_input(GPIOA, GPIO_PIN_8, presence_level);
    inputs_init();
    release_init();

    printf("boot selftest: contacts %s\n", presence_level == GPIO_PIN_RESET ? "closed" : "open");
    const unsigned failures = selftest_run();
    print_selftest_result();
    if (host_hal_motor_assertions() != 0u || host_hal_axis_command_calls() != 0u) {
        printf("boot actuator activity: %u motor SET writes, %u axis commands\n",
               host_hal_motor_assertions(), host_hal_axis_command_calls());
    }
    CHECK(failures == 0u);
    CHECK(host_hal_motor_assertions() == 0u);
    CHECK(host_hal_axis_command_calls() == 0u);
    for (unsigned reg = 0; reg < COMMAND_BLOCK_COUNT; reg++) {
        CHECK(regfile_get_holding((hreg_t)(COMMAND_BLOCK_START + reg)) == 0u);
    }

    /* A release queued during startup must not energize after the DIR delay. */
    host_hal_advance_tick(50u);
    release_step();
    CHECK(host_hal_motor_assertions() == 0u);
    CHECK(host_hal_axis_command_calls() == 0u);
}

void test_boot_run(void)
{
    boot_case(GPIO_PIN_RESET);
    boot_case(GPIO_PIN_SET);
}
