#ifndef TESTS_HOST_HAL_H
#define TESTS_HOST_HAL_H

#include "main.h"

void host_hal_reset(void);
void host_hal_set_tick(uint32_t tick);
void host_hal_advance_tick(uint32_t elapsed);
void host_hal_set_input(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void host_hal_set_link_up(uint8_t up);
unsigned host_hal_motor_assertions(void);
unsigned host_hal_legacy_assertions(void);
unsigned host_hal_motor_en_edges(void);
unsigned host_hal_direction_while_enabled(void);
unsigned host_hal_axis_command_calls(void);
const char *host_hal_uart_log(void);

#endif
