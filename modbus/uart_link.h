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

/* Поднимает слейв и разрешает приём по прерыванию. Звать после
 * MX_USART1_UART_Init() и после regfile_init(). */
void uart_link_init(void);

/* Вызывается из USART1_IRQHandler. Только читает байт и складывает в буфер. */
void uart_receive_isr(void);

/* Вызывается из основного цикла: переносит ошибки линии в счётчики и
 * прогоняет разбор кадра. */
void uart_link_task(void);

/* Для диагностики и для будущего regfile_update. */
mb_slave_t *uart_link_slave(void);

/* Была ли валидная транзакция за последние LINK_TIMEOUT_MS. Один ответ на
 * вопрос «связь есть» для телеметрии, команд и failsafe осей. */
uint8_t uart_link_is_up(void);

#endif /* MODBUS_UART_LINK_H */
