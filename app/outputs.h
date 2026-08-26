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


void outputs_step(void);


uint16_t outputs_actual(void);
uint8_t outputs_sgu_pulse_active(void);

#endif /* APP_OUTPUTS_H */
