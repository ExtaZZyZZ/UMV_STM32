/*
 * slave.h
 *
 *  Created on: 12 авг. 2026 г.
 *      Author: i.yakovlev
 */

#ifndef SLAVE_H_
#define SLAVE_H_
#include <stdint.h>
#include "pdu.h"

typedef struct {
	void(*uart_transmit)(const uint8_t *data, uint16_t len);
	void(*critical_enter)(void);
	void(*critical_exit)(void);
} mb_hw_ops_t;

typedef uint32_t (*mb_now_ms_fn)(void);

typedef struct {
    const mb_hw_ops_t *ops;
    mb_now_ms_fn       now_ms;
    uint8_t            slave_id;

    uint8_t  rx_buf[ADU_MAX_LEN];
    uint16_t rx_len;
    uint32_t last_rx_ms;
    volatile uint8_t rx_overflow;

    uint8_t  tx_buf[ADU_MAX_LEN];

    uint16_t crc_err;
    uint16_t frame_err;
    uint32_t last_valid_ms;
    uint8_t  link_ever_valid;
    uint8_t  fw_reply_pending;

    uint8_t  frame[ADU_MAX_LEN];
    uint16_t frame_len;
} mb_slave_t;

void mb_slave_init(mb_slave_t *s, const mb_hw_ops_t *ops,
                   mb_now_ms_fn now_ms, uint8_t slave_id);

void mb_slave_on_rx_byte(mb_slave_t *s, uint8_t byte);

void modbus_handler_task(mb_slave_t *s);

#endif /* SLAVE_H_ */
