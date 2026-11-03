#include "host_hal.h"
#include "test_support.h"

#include "command.h"
#include "regfile.h"

/* Include the real FSM once so timing changes need no production test API. */
#include "../../app/release.c"

static const uint32_t motion_ms = RELEASE_DIR_SETTLE_MS + RELEASE_TRAVEL_MS;

/* Physical wiring is the test oracle, independent of the production labels. */
static GPIO_PinState enable_level(unsigned unit)
{
    return HAL_GPIO_ReadPin(GPIOA, unit == 0u ? GPIO_PIN_15 : GPIO_PIN_10);
}

static GPIO_PinState direction_level(unsigned unit)
{
    return unit == 0u
        ? HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)
        : HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
}

static void set_raw_presence(unsigned unit, unsigned present)
{
    const unsigned closed = RELEASE_PRESENT_WHEN_CLOSED ? present : !present;
    const GPIO_PinState level = closed ? GPIO_PIN_RESET : GPIO_PIN_SET;

    /* inputs.c uses active-low contacts; the presence interpretation is provisional. */
    if (unit == 0u) {
        host_hal_set_input(GPIOD, GPIO_PIN_1, level);
    } else {
        host_hal_set_input(GPIOA, GPIO_PIN_8, level);
    }
}

static void arrange_presence(unsigned unit, unsigned present)
{
    set_raw_presence(unit, present);
    inputs_init(); /* Model an input that was already debounced before this step. */
}

static void fixture_at(uint32_t now)
{
    host_hal_reset();
    host_hal_set_tick(now);
    set_raw_presence(0u, 1u);
    set_raw_presence(1u, 1u);
    inputs_init();
    release_init();
}

static void step_at(uint32_t now)
{
    host_hal_set_tick(now);
    release_step();
}

static uint16_t dispatch_release(uint16_t seq, uint16_t unit, uint16_t reserved)
{
    (void)regfile_write_holding(HREG_CMD_SEQ, seq);
    (void)regfile_write_holding(HREG_CMD_CODE, CMD_CODE_RELEASE);
    (void)regfile_write_holding(HREG_CMD_ARG0, unit);
    (void)regfile_write_holding(HREG_CMD_ARG1, reserved);
    command_step();

    uint16_t value = 0u;
    (void)regfile_read_input(IREG_ACK_SEQ, &value);
    CHECK(value == seq);
    (void)regfile_read_input(IREG_ACK_RESULT, &value);
    return value;
}

static void check_motor_safety(void)
{
    CHECK(host_hal_direction_while_enabled() == 0u);
    CHECK(host_hal_axis_command_calls() == 0u);
}

static uint32_t finish_extension(uint32_t start)
{
    fixture_at(start);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    step_at(start + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(start + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    return start + motion_ms;
}

static void test_init_and_invalid_unit_do_not_move(void)
{
    fixture_at(1000u);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    CHECK(host_hal_motor_assertions() == 0u);
    step_at(1000u + motion_ms + RELEASE_CONFIRM_TIMEOUT_MS);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(releases[1].state == RELEASE_STATE_IDLE);
    CHECK(host_hal_motor_assertions() == 0u);

    const GPIO_PinState dir0 = direction_level(0u);
    const GPIO_PinState dir1 = direction_level(1u);
    CHECK(release_cmd_start(RELEASE_COUNT) == ACK_RESULT_BAD_ARG);
    CHECK(release_cmd_start(UINT16_MAX) == ACK_RESULT_BAD_ARG);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == dir0);
    CHECK(direction_level(1u) == dir1);
    CHECK(host_hal_motor_assertions() == 0u);
    check_motor_safety();
}

static void test_reinit_deasserts_already_enabled_drivers(void)
{
    fixture_at(1000u);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15 | GPIO_PIN_10, GPIO_PIN_SET);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(enable_level(1u) == GPIO_PIN_SET);
    release_init();
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    step_at(1000u + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(releases[1].state == RELEASE_STATE_IDLE);
    check_motor_safety();
}

static void test_absence_refuses_only_the_selected_channel(void)
{
    for (unsigned absent = 0u; absent < RELEASE_COUNT; ++absent) {
        fixture_at(1000u);
        arrange_presence(absent, 0u);
        CHECK(release_cmd_start((uint16_t)absent) == ACK_RESULT_REFUSED_SAFETY);
        CHECK(host_hal_motor_assertions() == 0u);
        CHECK(enable_level(0u) == GPIO_PIN_RESET);
        CHECK(enable_level(1u) == GPIO_PIN_RESET);
        CHECK(release_cmd_start((uint16_t)(1u - absent)) == ACK_RESULT_OK);
        step_at(1000u + RELEASE_DIR_SETTLE_MS);
        CHECK(enable_level(absent) == GPIO_PIN_RESET);
        CHECK(enable_level(1u - absent) == GPIO_PIN_SET);
        check_motor_safety();
    }
}

static void test_presence_input_requires_twenty_samples(void)
{
    fixture_at(1000u);
    const uint16_t before = inputs_bits();
    set_raw_presence(0u, 0u);
    for (unsigned sample = 0u; sample < 19u; ++sample) {
        inputs_step();
    }
    CHECK(inputs_bits() == before);
    inputs_step();
    CHECK(inputs_bits() == (uint16_t)(before ^ (1u << IN_RELEASE1_DEPLOYED)));
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_SAFETY);
    CHECK(release_cmd_start(1u) == ACK_RESULT_OK);
    CHECK(host_hal_motor_en_edges() == 0u);
}

static void test_full_cycle(uint32_t start)
{
    const uint32_t extended = start + motion_ms;
    const uint32_t absent_at = extended + 1u;
    const uint32_t returning = absent_at + RELEASE_RETURN_DELAY_MS;
    fixture_at(start);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_EXTEND_LEVEL);

    step_at(start + RELEASE_DIR_SETTLE_MS - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    step_at(start + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    arrange_presence(0u, 0u);
    step_at(start + RELEASE_DIR_SETTLE_MS + RELEASE_TRAVEL_MS / 2u);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_BUSY);
    CHECK(release_cmd_start(UINT16_MAX) == ACK_RESULT_BAD_ARG);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(extended - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(extended);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(host_hal_motor_en_edges() == 1u);

    step_at(absent_at);
    CHECK(releases[0].state == RELEASE_STATE_RETURN_DELAY);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    step_at(absent_at + RELEASE_RETURN_DELAY_MS / 2u);
    CHECK(releases[0].state == RELEASE_STATE_RETURN_DELAY);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_BUSY);
    step_at(returning - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_EXTEND_LEVEL);
    step_at(returning);
    CHECK(releases[0].state == RELEASE_STATE_RETRACTING);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_RETRACT_LEVEL);
    step_at(returning + RELEASE_DIR_SETTLE_MS - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    step_at(returning + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_BUSY);
    step_at(returning + motion_ms - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(returning + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    CHECK(host_hal_motor_en_edges() == 2u);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_SAFETY);
    arrange_presence(0u, 1u);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    check_motor_safety();
}

static void test_missing_confirmation_latches_fault(uint32_t start)
{
    const uint32_t extended = finish_extension(start);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS / 2u);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_BUSY);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS - 1u);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS);
    CHECK(releases[0].state == RELEASE_STATE_FAULT);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_SAFETY);
    arrange_presence(0u, 0u);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS + RELEASE_RETURN_DELAY_MS + motion_ms);
    CHECK(releases[0].state == RELEASE_STATE_FAULT);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    arrange_presence(0u, 1u);
    CHECK(release_cmd_start(0u) == ACK_RESULT_REFUSED_SAFETY);
    CHECK(host_hal_motor_en_edges() == 1u);
    check_motor_safety();
}

static void test_reappearing_presence_keeps_original_confirmation_deadline(void)
{
    const uint32_t extended = finish_extension(1000u);
    const uint32_t cancel_offset =
        (RELEASE_RETURN_DELAY_MS < RELEASE_CONFIRM_TIMEOUT_MS
            ? RELEASE_RETURN_DELAY_MS : RELEASE_CONFIRM_TIMEOUT_MS) / 2u;
    arrange_presence(0u, 0u);
    step_at(extended + 1u);
    CHECK(releases[0].state == RELEASE_STATE_RETURN_DELAY);
    arrange_presence(0u, 1u);
    step_at(extended + cancel_offset);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_EXTEND_LEVEL);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS - 1u);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    step_at(extended + RELEASE_CONFIRM_TIMEOUT_MS);
    CHECK(releases[0].state == RELEASE_STATE_FAULT);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(host_hal_motor_en_edges() == 1u);
    check_motor_safety();
}

static void test_late_confirmation_still_gets_full_return_pause(void)
{
    const uint32_t extended = finish_extension(1000u);
    const uint32_t confirmed = extended + RELEASE_CONFIRM_TIMEOUT_MS - 1u;
    const uint32_t returning = confirmed + RELEASE_RETURN_DELAY_MS;
    arrange_presence(0u, 0u);
    step_at(confirmed);
    CHECK(releases[0].state == RELEASE_STATE_RETURN_DELAY);
    step_at(returning - 1u);
    CHECK(releases[0].state == RELEASE_STATE_RETURN_DELAY);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_EXTEND_LEVEL);
    step_at(returning);
    CHECK(releases[0].state == RELEASE_STATE_RETRACTING);
    CHECK(direction_level(0u) == RELEASE_RETRACT_LEVEL);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    step_at(returning + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(returning + motion_ms);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    check_motor_safety();
}

static void test_channels_finish_both_strokes_independently(void)
{
    const uint32_t start = 1000u;
    const uint32_t stagger = RELEASE_TRAVEL_MS / 2u < RELEASE_DIR_SETTLE_MS
        ? RELEASE_TRAVEL_MS / 2u : RELEASE_DIR_SETTLE_MS;
    const uint32_t extended0 = start + motion_ms;
    const uint32_t extended1 = extended0 + stagger;
    const uint32_t returning0 = extended0 + 1u + RELEASE_RETURN_DELAY_MS;
    const uint32_t returning1 = returning0 + stagger;
    fixture_at(start);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    host_hal_set_tick(start + stagger);
    CHECK(release_cmd_start(1u) == ACK_RESULT_OK);
    step_at(start + stagger + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(enable_level(1u) == GPIO_PIN_SET);
    step_at(extended0);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_SET);
    arrange_presence(0u, 0u);
    step_at(extended0 + 1u);
    step_at(extended1);
    CHECK(releases[1].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    arrange_presence(1u, 0u);
    step_at(extended1 + 1u);
    step_at(returning0);
    CHECK(direction_level(0u) == RELEASE_RETRACT_LEVEL);
    CHECK(direction_level(1u) == RELEASE_EXTEND_LEVEL);
    step_at(returning1);
    CHECK(direction_level(1u) == RELEASE_RETRACT_LEVEL);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    step_at(returning0 + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    step_at(returning1 + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    CHECK(enable_level(1u) == GPIO_PIN_SET);
    step_at(returning0 + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(enable_level(1u) == GPIO_PIN_SET);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    step_at(returning1 + motion_ms);
    CHECK(enable_level(1u) == GPIO_PIN_RESET);
    CHECK(releases[1].state == RELEASE_STATE_IDLE);
    CHECK(host_hal_motor_en_edges() == 4u);
    check_motor_safety();
}

static void test_expired_first_step_never_pulses_enable(void)
{
    const uint32_t start = 1000u;
    const uint32_t returning = start + motion_ms + RELEASE_RETURN_DELAY_MS;
    fixture_at(start);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    arrange_presence(0u, 0u);
    step_at(start + motion_ms);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(host_hal_motor_en_edges() == 0u);
    step_at(start + motion_ms);
    step_at(returning);
    CHECK(releases[0].state == RELEASE_STATE_RETRACTING);
    step_at(returning + motion_ms);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(host_hal_motor_en_edges() == 0u);
    check_motor_safety();
}

static void test_pi_link_loss_does_not_interrupt_a_cycle(void)
{
    const uint32_t start = 1000u;
    const uint32_t extended = start + motion_ms;
    const uint32_t returning = extended + 1u + RELEASE_RETURN_DELAY_MS;
    fixture_at(start);
    host_hal_set_link_up(1u);
    CHECK(release_cmd_start(0u) == ACK_RESULT_OK);
    step_at(start + RELEASE_DIR_SETTLE_MS);
    host_hal_set_link_up(0u);
    step_at(extended - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(extended);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    arrange_presence(0u, 0u);
    step_at(extended + 1u);
    step_at(returning);
    step_at(returning + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(returning + motion_ms - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_SET);
    step_at(returning + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_IDLE);
    CHECK(host_hal_motor_en_edges() == 2u);
    check_motor_safety();
}

static void test_release_dispatches_to_timed_drivers(void)
{
    const uint32_t start = 1000u;
    const uint32_t extended = start + motion_ms;
    const uint32_t returning = extended + 1u + RELEASE_RETURN_DELAY_MS;
    fixture_at(start);
    regfile_init();
    command_init();

    CHECK(dispatch_release(1u, 0u, 500u) == ACK_RESULT_BAD_ARG);
    CHECK(dispatch_release(2u, 0u, 1u) == ACK_RESULT_BAD_ARG);
    CHECK(dispatch_release(3u, RELEASE_COUNT, 0u) == ACK_RESULT_BAD_ARG);
    arrange_presence(0u, 0u);
    CHECK(dispatch_release(4u, 0u, 0u) == ACK_RESULT_REFUSED_SAFETY);
    CHECK(host_hal_motor_assertions() == 0u);
    CHECK(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15) == GPIO_PIN_RESET);

    arrange_presence(0u, 1u);
    host_hal_set_link_up(1u);
    CHECK(dispatch_release(5u, 0u, 0u) == ACK_RESULT_OK);
    CHECK(releases[0].state == RELEASE_STATE_EXTENDING);
    CHECK(releases[1].state == RELEASE_STATE_IDLE);
    CHECK(direction_level(0u) == RELEASE_EXTEND_LEVEL);
    CHECK(dispatch_release(6u, 1u, 0u) == ACK_RESULT_OK);
    CHECK(releases[1].state == RELEASE_STATE_EXTENDING);
    CHECK(enable_level(0u) == GPIO_PIN_RESET && enable_level(1u) == GPIO_PIN_RESET);

    host_hal_set_tick(start + RELEASE_DIR_SETTLE_MS / 2u);
    CHECK(dispatch_release(6u, 1u, 0u) == ACK_RESULT_OK);
    CHECK(dispatch_release(7u, 0u, 0u) == ACK_RESULT_REFUSED_BUSY);
    CHECK(host_hal_legacy_assertions() == 0u);
    step_at(start + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET && enable_level(1u) == GPIO_PIN_SET);
    host_hal_set_link_up(0u);
    command_step();
    CHECK(enable_level(0u) == GPIO_PIN_SET && enable_level(1u) == GPIO_PIN_SET);
    step_at(extended - 1u);
    CHECK(enable_level(0u) == GPIO_PIN_SET && enable_level(1u) == GPIO_PIN_SET);
    step_at(extended);
    CHECK(enable_level(0u) == GPIO_PIN_RESET && enable_level(1u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_WAIT_RELEASE);
    CHECK(releases[1].state == RELEASE_STATE_WAIT_RELEASE);
    arrange_presence(0u, 0u);
    arrange_presence(1u, 0u);
    step_at(extended + 1u);
    step_at(returning);
    CHECK(enable_level(0u) == GPIO_PIN_RESET && enable_level(1u) == GPIO_PIN_RESET);
    CHECK(direction_level(0u) == RELEASE_RETRACT_LEVEL);
    CHECK(direction_level(1u) == RELEASE_RETRACT_LEVEL);
    step_at(returning + RELEASE_DIR_SETTLE_MS);
    CHECK(enable_level(0u) == GPIO_PIN_SET && enable_level(1u) == GPIO_PIN_SET);
    step_at(returning + motion_ms);
    CHECK(enable_level(0u) == GPIO_PIN_RESET && enable_level(1u) == GPIO_PIN_RESET);
    CHECK(releases[0].state == RELEASE_STATE_IDLE && releases[1].state == RELEASE_STATE_IDLE);
    CHECK(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15) == GPIO_PIN_RESET);
    CHECK(host_hal_legacy_assertions() == 0u);
    CHECK(host_hal_motor_en_edges() == 4u);
    check_motor_safety();
}

void test_release_run(void)
{
    test_init_and_invalid_unit_do_not_move();
    test_reinit_deasserts_already_enabled_drivers();
    test_absence_refuses_only_the_selected_channel();
    test_presence_input_requires_twenty_samples();
    test_full_cycle(1000u);
    test_missing_confirmation_latches_fault(1000u);
    test_reappearing_presence_keeps_original_confirmation_deadline();
    test_late_confirmation_still_gets_full_return_pause();
    test_channels_finish_both_strokes_independently();
    test_expired_first_step_never_pulses_enable();
    test_pi_link_loss_does_not_interrupt_a_cycle();

    /* Cross uint32_t rollover during the guard, return pause, and return stroke. */
    test_full_cycle(UINT32_MAX - RELEASE_DIR_SETTLE_MS / 2u);
    test_full_cycle(UINT32_MAX - motion_ms - RELEASE_RETURN_DELAY_MS / 2u);
    test_full_cycle(UINT32_MAX - motion_ms - 1u - RELEASE_RETURN_DELAY_MS
                    - RELEASE_DIR_SETTLE_MS - RELEASE_TRAVEL_MS / 2u);
    test_missing_confirmation_latches_fault(UINT32_MAX - motion_ms
                                            - RELEASE_CONFIRM_TIMEOUT_MS / 2u);
    test_release_dispatches_to_timed_drivers();
}
