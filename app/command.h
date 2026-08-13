#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include <stdint.h>

void command_init(void);

/* 50 Гц: дедуп по cmd_seq, исполнение, квитанция в ack_seq / ack_result. */
void command_step(void);

/* 1 кГц: снятие импульса замка круга. Безусловно, независимо от связи. */
void command_pulse_step(void);

/* Идёт ли сейчас импульс замка. Нужно тестам и для диагностики. */
uint8_t command_buoy_active(void);

#endif /* APP_COMMAND_H */
