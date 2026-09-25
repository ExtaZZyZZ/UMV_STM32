/*
 * uart_link.h — привязка слейва Modbus к USART1.
 *
 * Единственное место в проекте, где есть HAL и обращения к регистрам.
 * Всё остальное — crc16, pdu, regfile, slave — от железа не зависит и
 * проверяется заглушками.
 */
#ifndef MODBUS_UART_LINK_H
#define MODBUS_UART_LINK_H

#include "slave.h"


void uart_link_init(void);

void enc_link_init(void);

void enc_receive_isr(void);

void uart_receive_isr(void);


void uart_link_task(void);


mb_slave_t *uart_link_slave(void);


uint8_t uart_link_is_up(void);

#endif /* MODBUS_UART_LINK_H */
