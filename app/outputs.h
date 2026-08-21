/*
 * outputs.h — дискретные выходы: фары, балка, синий проблесковый СГУ и
 * подводные огни (четыре светильника на одном общем канале).
 *
 * Маска приходит от мастера в регистр outputs (FC06), фактическое состояние
 * уходит обратно в do_actual (FC04).
 */
#ifndef APP_OUTPUTS_H
#define APP_OUTPUTS_H

#include <stdint.h>

void outputs_init(void);

/* 50 Гц. Чаще незачем: 20 мс задержки на включение лампы незаметны. */
void outputs_step(void);

/* Текущая маска, поданная на ноги. Для тестов и диагностики. */
uint16_t outputs_actual(void);

#endif /* APP_OUTPUTS_H */
