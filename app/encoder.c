/*
 * encoder.c
 *
 *  Created on: 22 сент. 2026 г.
 *      Author: i.yakovlev
 */

#include "encoder.h"
#include "pdu.h"
#define ENC_ADDR        1u
#define ENC_FC          0x03u
#define ENC_REPLY_LEN   7u
#define ENC_TIMEOUT_MS 50u
#define ENC_GAP_MS      4u
#define ENC_LOST_MS   200u

typedef enum {ENC_SEND = 0, ENC_RX, ENC_GAP } enc_state_t;

static void (*enc_transmit)(const uint8_t *, uint16_t);
static uint32_t (*enc_now)(void);
static uint8_t req[8];

static uint8_t rx[ENC_REPLY_LEN];
static volatile uint8_t rx_len;

static enc_state_t state;
static uint32_t    mark;
static int32_t     counts;
static uint32_t    last_ok_ms;
static uint8_t     ever_ok;
static uint16_t    err_count;

static void enc_accept(void)
{
	uint16_t pdu_len = 0u;
	if(adu_check(rx, ENC_REPLY_LEN, ENC_ADDR, &pdu_len) != ADU_OK
			|| rx[1] != ENC_FC || rx[2] != 2u) {
		err_count++;
		return;
	}
	counts = (int32_t)rd_u16be(&rx[3]);
	last_ok_ms = enc_now();
	ever_ok = 1u;
}

void encoder_init(void(*transmit)(const uint8_t*, uint16_t), uint32_t(*now_ms)(void))
{
	enc_transmit = transmit;
	enc_now = now_ms;
	req[0] = ENC_ADDR;
	req[1] = ENC_FC;
	wr_u16be(&req[2], 0x0000u);
	wr_u16be(&req[4], 0x0001u);
	(void)adu_wrap(req, 6u);
	rx_len = 0u;
	state = ENC_SEND;
	mark = now_ms();
	counts = 0;
    last_ok_ms = 0u;
    ever_ok    = 0u;
    err_count  = 0u;
}

void encoder_on_rx_byte(uint8_t byte)
{
	if(rx_len < sizeof(rx)) {
		rx[rx_len] = byte;
		rx_len++;
	}
}

void encoder_step(void)
{
	const uint32_t now = enc_now();
	switch (state) {
	case ENC_SEND:
		rx_len = 0u;
		enc_transmit(req, sizeof(req));
		mark = now;
		state = ENC_RX;
		break;
	case ENC_RX:
		if (rx_len >= ENC_REPLY_LEN) {
			enc_accept();
			mark = now;
			state = ENC_GAP;
		} else if((now - mark) >= ENC_TIMEOUT_MS) {
			err_count ++;
			mark = now;
			state = ENC_GAP;
		}
		break;
	case ENC_GAP:
		if ((now - mark) >= ENC_GAP_MS) {
			state = ENC_SEND;
		}
		break;
	}
}

int32_t encoder_counts(void) {
	return counts;
}

uint8_t encoder_is_up(void)
{
    return (uint8_t)(ever_ok && ((enc_now() - last_ok_ms) < ENC_LOST_MS));
}

uint16_t encoder_err_count(void)
{
    return err_count;
}
