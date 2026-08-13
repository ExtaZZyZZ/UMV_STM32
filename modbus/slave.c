/*
 * slave.c
 *
 *  Created on: 12 авг. 2026 г.
 *      Author: i.yakovlev
 */
#include "slave.h"
#include <string.h>
#include "regfile.h"

#define MB_FRAME_GAP_TICKS 2u
#define MB_FC_READ_HOLDING   0x03u
#define MB_FC_READ_INPUT     0x04u
#define MB_FC_WRITE_SINGLE   0x06u
#define MB_FC_WRITE_MULTIPLE 0x10u
#define MB_EX_ILLEGAL_FUNCTION 0x01u
#define MB_EX_ILLEGAL_ADDRESS  0x02u
#define MB_EX_ILLEGAL_VALUE    0x03u

static void handle_read_holding(mb_slave_t *s);
static void handle_read_input(mb_slave_t *s);
static void handle_write_single(mb_slave_t *s);
static void handle_write_multiple(mb_slave_t *s);

static void mb_send_exception(mb_slave_t *s, uint8_t function, uint8_t code){
	s->tx_buf[0] = s->slave_id;
	s->tx_buf[1] = function | 0x80;
	s->tx_buf[2] = code;
	uint16_t len = adu_wrap(s->tx_buf, 3);
	s->ops->uart_transmit(s->tx_buf, len);
}

void mb_slave_init(mb_slave_t *s, const mb_hw_ops_t *ops, mb_now_ms_fn now_ms, uint8_t slave_id)
{
	memset(s, 0, sizeof(*s));
	s->ops = ops;
	s->now_ms = now_ms;
	s->slave_id = slave_id;
	s->last_rx_ms = now_ms();
	s->last_valid_ms = now_ms();
}

void mb_slave_on_rx_byte(mb_slave_t *s, uint8_t byte)
{
	if (s->rx_len < sizeof(s->rx_buf)) {
		s->rx_buf[s->rx_len] = byte;
		s->rx_len += 1;
	}
	else {
		s->rx_overflow = 1;
		s->rx_len = 0;
	}
	s->last_rx_ms = s->now_ms();
}

void modbus_handler_task(mb_slave_t *s)
{
	if(s->rx_overflow) {
		s->rx_overflow = 0;
		s->frame_err += 1;
	}

	if(s->rx_len == 0) return;
	if((s->now_ms() - s->last_rx_ms) < MB_FRAME_GAP_TICKS) return;

	s->ops->critical_enter(); //отключаем прерывания для защиты от гонки
	s->frame_len = s->rx_len;
	memcpy(s->frame, s->rx_buf, s->frame_len);
	s->rx_len = 0;
	s->ops->critical_exit();

	uint16_t pdu_len = 0;
	adu_status_t status = adu_check(s->frame, s->frame_len, s->slave_id, &pdu_len);
	if (status == ADU_ERR_CRC) {
		s->crc_err += 1;
		return;
	}
	else if(status != ADU_OK) {
		s->frame_err += 1;
		return;
	}
	s->last_valid_ms    = s->now_ms();
	s->link_ever_valid  = 1u;

	switch (s->frame[1]) {
	    case MB_FC_READ_HOLDING:
	        handle_read_holding(s);
	        break;

	    case MB_FC_READ_INPUT:
	        handle_read_input(s);
	        break;

	    case MB_FC_WRITE_SINGLE:
	        handle_write_single(s);
	        break;

	    case MB_FC_WRITE_MULTIPLE:
	        handle_write_multiple(s);
	        break;

	    default:
	        mb_send_exception(s, s->frame[1], MB_EX_ILLEGAL_FUNCTION);
	        break;
	}
}

static void handle_read_input(mb_slave_t *s)
{
	if(s->frame_len != 8) {
		mb_send_exception(s, MB_FC_READ_INPUT, MB_EX_ILLEGAL_VALUE);
		return;
	}
	uint16_t start = rd_u16be(&s->frame[2]);
	uint16_t qty = rd_u16be(&s->frame[4]);
	if (qty == 0 || qty > IREG_COUNT) {
		mb_send_exception(s, MB_FC_READ_INPUT, MB_EX_ILLEGAL_VALUE);
		return;
	}
    if (start + qty > IREG_COUNT) {
        mb_send_exception(s, MB_FC_READ_INPUT, MB_EX_ILLEGAL_ADDRESS);
        return;
    }
	s->tx_buf[0] = s->slave_id;
	s->tx_buf[1] = MB_FC_READ_INPUT;
	s->tx_buf[2] = qty * 2;
	for(int i = 0; i < qty; i++) {
		uint16_t v = 0;
		regfile_read_input(start + i, &v);
		wr_u16be(&s->tx_buf[3 + i * 2], v);
	}
	uint16_t len = adu_wrap(s->tx_buf, 3 + qty * 2);
	s->ops->uart_transmit(s->tx_buf, len);
}
static void handle_read_holding(mb_slave_t *s)
{
	if(s->frame_len != 8) {
		mb_send_exception(s, MB_FC_READ_HOLDING, MB_EX_ILLEGAL_VALUE);
		return;
	}
	uint16_t start = rd_u16be(&s->frame[2]);
	uint16_t qty = rd_u16be(&s->frame[4]);
	if (qty == 0 || qty > HREG_COUNT) {
		mb_send_exception(s, MB_FC_READ_HOLDING, MB_EX_ILLEGAL_VALUE);
		return;
	}
    if (start + qty > HREG_COUNT) {
        mb_send_exception(s, MB_FC_READ_HOLDING, MB_EX_ILLEGAL_ADDRESS);
        return;
    }
	s->tx_buf[0] = s->slave_id;
	s->tx_buf[1] = MB_FC_READ_HOLDING;
	s->tx_buf[2] = qty * 2;
	for(int i = 0; i < qty; i++) {
		uint16_t v = 0;
		regfile_read_holding(start + i, &v);
		wr_u16be(&s->tx_buf[3 + i * 2], v);
	}
	uint16_t len = adu_wrap(s->tx_buf, 3 + qty * 2);
	s->ops->uart_transmit(s->tx_buf, len);
}



static void handle_write_single(mb_slave_t *s)
{
	if (s->frame_len != 8) {
		mb_send_exception(s, MB_FC_WRITE_SINGLE, MB_EX_ILLEGAL_VALUE);
		return;
	}

	uint16_t addr  = rd_u16be(&s->frame[2]);
	uint16_t value = rd_u16be(&s->frame[4]);

	if (!regfile_write_holding(addr, value)) {
		mb_send_exception(s, MB_FC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDRESS);
		return;
	}

	memcpy(s->tx_buf, s->frame, 6u);
	uint16_t len = adu_wrap(s->tx_buf, 6u);
	s->ops->uart_transmit(s->tx_buf, len);
}

static void handle_write_multiple(mb_slave_t *s)
{

	if (s->frame_len < 9u) {
		mb_send_exception(s, MB_FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_VALUE);
		return;
	}

	uint16_t start      = rd_u16be(&s->frame[2]);
	uint16_t qty        = rd_u16be(&s->frame[4]);
	uint8_t  byte_count = s->frame[6];

	if (qty == 0u || qty > HREG_COUNT || byte_count != qty * 2u) {
		mb_send_exception(s, MB_FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_VALUE);
		return;
	}
	if (s->frame_len != 9u + qty * 2u) {
		mb_send_exception(s, MB_FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_VALUE);
		return;
	}
	if (start + qty > HREG_COUNT) {
		mb_send_exception(s, MB_FC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDRESS);
		return;
	}

	for (uint16_t i = 0; i < qty; i++) {
		(void)regfile_write_holding(start + i, rd_u16be(&s->frame[7 + i * 2]));
	}

	s->tx_buf[0] = s->slave_id;
	s->tx_buf[1] = MB_FC_WRITE_MULTIPLE;
	wr_u16be(&s->tx_buf[2], start);
	wr_u16be(&s->tx_buf[4], qty);
	uint16_t len = adu_wrap(s->tx_buf, 6u);
	s->ops->uart_transmit(s->tx_buf, len);
}


