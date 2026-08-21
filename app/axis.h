/*
 * axis.h — две мачты СПМ на приводах HAKIWO через мосты BTS7960.
 *
 * Управление пока разомкнутое, по времени: энкодера BRT38 в прошивке нет.
 * Следствие — прерванный ход делает положение неизвестным до AXIS_HOME.
 */
#ifndef APP_AXIS_H
#define APP_AXIS_H

#include <stdint.h>

void axis_init(void);

/* 200 Гц: разгон, отсчёт хода, перенос состояния в телеметрию. */
void axis_step(void);

/* 1 кГц: снятие моста при потере связи. Безусловно, как и снятие импульса замка. */
void axis_failsafe_step(void);

/* Команды. Возвращают ack_result_t; проверку границ делает вызывающий. */
uint16_t axis_cmd_goto(uint16_t axis, uint16_t step);
uint16_t axis_cmd_stop(uint16_t axis);
uint16_t axis_cmd_home(uint16_t axis);

#endif /* APP_AXIS_H */
