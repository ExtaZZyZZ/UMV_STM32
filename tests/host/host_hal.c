#include "host_hal.h"

#include "axis.h"
#include "board_registers.h"
#include "uart_link.h"

#include <string.h>

GPIO_TypeDef host_gpio_a, host_gpio_b, host_gpio_c;
GPIO_TypeDef host_gpio_d, host_gpio_e, host_gpio_f;
UART_HandleTypeDef huart1;

static uint32_t tick_ms;
static uint8_t link_up;
static unsigned motor_assertions, motor_en_edges, direction_while_enabled;
static unsigned legacy_assertions;
static unsigned axis_command_calls;
static mb_slave_t link_slave;
static char uart_log[32768];
static size_t uart_log_length;

void host_hal_reset(void)
{
    GPIO_TypeDef *const ports[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF };
    for (unsigned i = 0; i < sizeof(ports) / sizeof(ports[0]); i++) {
        memset(ports[i], 0, sizeof(*ports[i]));
    }
    tick_ms = 1000u;
    link_up = 0u;
    motor_assertions = motor_en_edges = direction_while_enabled = 0u;
    legacy_assertions = 0u;
    axis_command_calls = 0u;
    memset(&link_slave, 0, sizeof(link_slave));
    uart_log_length = 0u;
    uart_log[0] = '\0';
}

void host_hal_set_tick(uint32_t tick) { tick_ms = tick; }
void host_hal_advance_tick(uint32_t elapsed) { tick_ms += elapsed; }
void host_hal_set_link_up(uint8_t up) { link_up = up; }
unsigned host_hal_motor_assertions(void) { return motor_assertions; }
unsigned host_hal_legacy_assertions(void) { return legacy_assertions; }
unsigned host_hal_motor_en_edges(void) { return motor_en_edges; }
unsigned host_hal_direction_while_enabled(void) { return direction_while_enabled; }
unsigned host_hal_axis_command_calls(void) { return axis_command_calls; }
const char *host_hal_uart_log(void) { return uart_log; }
uint32_t HAL_GetTick(void) { return tick_ms; }

void host_hal_set_input(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) port->IDR |= pin;
    else port->IDR &= (uint16_t)~pin;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    uint16_t motor_mask = 0u;
    uint16_t en_mask = 0u;
    if (port == GPIOA) motor_mask = en_mask = GPIO_PIN_15 | GPIO_PIN_10;
    if (port == GPIOB) motor_mask = GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_9;
    if (port == GPIOC) motor_mask = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    if (port == GPIOD) motor_mask = GPIO_PIN_0;
    if (port == GPIOE) motor_mask = GPIO_PIN_10;
    if (port == GPIOF) motor_mask = GPIO_PIN_2;

    /* Confirmed wiring is independent of the production label definitions. */
    if ((port == GPIOE && (pin & GPIO_PIN_10) && (GPIOA->ODR & GPIO_PIN_15)) ||
        (port == GPIOD && (pin & GPIO_PIN_0) && (GPIOA->ODR & GPIO_PIN_10))) {
        direction_while_enabled++;
    }
    if (state == GPIO_PIN_SET) {
        if (port == GPIOB && (pin & (GPIO_PIN_13 | GPIO_PIN_15))) legacy_assertions++;
        for (unsigned bit = 0; bit < 16u; bit++) {
            const uint16_t mask = (uint16_t)(1u << bit);
            if (pin & motor_mask & mask) motor_assertions++;
            if (pin & en_mask & mask & (uint16_t)~port->ODR) motor_en_edges++;
        }
        port->ODR |= pin;
    } else {
        port->ODR &= (uint16_t)~pin;
    }
    host_hal_set_input(port, pin, state);
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
                                    const uint8_t *data, uint16_t length,
                                    uint32_t timeout)
{
    (void)uart;
    (void)timeout;
    size_t count = length;
    const size_t available = sizeof(uart_log) - uart_log_length - 1u;
    if (count > available) count = available;
    memcpy(uart_log + uart_log_length, data, count);
    uart_log_length += count;
    uart_log[uart_log_length] = '\0';
    return HAL_OK;
}

uint8_t uart_link_is_up(void) { return link_up; }
mb_slave_t *uart_link_slave(void) { return &link_slave; }

uint16_t axis_cmd_goto(uint16_t axis, uint16_t step)
{
    (void)axis;
    (void)step;
    axis_command_calls++;
    return ACK_RESULT_OK;
}

uint16_t axis_cmd_stop(uint16_t axis)
{
    (void)axis;
    axis_command_calls++;
    return ACK_RESULT_OK;
}

uint16_t axis_cmd_home(uint16_t axis)
{
    (void)axis;
    axis_command_calls++;
    return ACK_RESULT_OK;
}

uint16_t axis_cmd_calibrate(uint16_t axis)
{
    (void)axis;
    axis_command_calls++;
    return ACK_RESULT_OK;
}
