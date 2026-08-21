#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include <stdint.h>

void command_init(void);

/* 50 Гц: дедуп по cmd_seq, исполнение, квитанция в ack_seq / ack_result. */
void command_step(void);

/* 1 кГц: снятие импульсов замков ПС. Безусловно, независимо от связи. */
void command_pulse_step(void);

/* Идёт ли сейчас импульс замка устройства `unit`. Тестам и для диагностики.
 * Номер вне диапазона даёт 0, а не выход за массив. */
uint8_t command_release_active(uint8_t unit);

#endif /* APP_COMMAND_H */
