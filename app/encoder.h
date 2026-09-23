/*
 * encoder.h
 *
 *  Created on: 22 сент. 2026 г.
 *      Author: i.yakovlev
 */

#ifndef ENCODER_H_
#define ENCODER_H_
#include <stdint.h>

void encoder_init(void (*transmit)(const uint8_t *, uint16_t),
                  uint32_t (*now_ms)(void));


void encoder_step(void);


void encoder_on_rx_byte(uint8_t byte);


int32_t  encoder_counts(void);
uint8_t  encoder_is_up(void);
uint16_t encoder_err_count(void);


#endif /* ENCODER_H_ */
