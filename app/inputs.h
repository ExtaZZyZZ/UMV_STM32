/*
 * inputs.h — дискретные входы: концевики спуска ПС и концевики их замков.
 *
 * Уровень уходит в di_bits (FC04) после фильтрации дребезга.
 */
#ifndef APP_INPUTS_H
#define APP_INPUTS_H

#include <stdint.h>

void inputs_init(void);

/* 1 кГц: выборка ног и счётчики дребезга. */
void inputs_step(void);

/* Отфильтрованная маска, биты по таблице in_t. */
uint16_t inputs_bits(void);

#endif /* APP_INPUTS_H */
