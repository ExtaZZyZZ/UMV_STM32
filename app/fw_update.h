/*
 * fw_update.h
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: i.yakovlev
 */

#ifndef APP_FW_UPDATE_H
#define APP_FW_UPDATE_H

#include <stdbool.h>
#include <stdint.h>

#define FW_UPDATE_REG_BASE 0x0100u
#define FW_UPDATE_IREG_COUNT 16u
#define FW_UPDATE_HEADER_WORDS 4u

typedef enum {
    FW_UPDATE_OK,
    FW_UPDATE_REJECTED,
    FW_UPDATE_FAILED,
    FW_UPDATE_DEFERRED,
} fw_update_reply_t;

void fw_update_init(uint32_t now);
void fw_update_step(uint32_t now, bool link_up, bool tx_idle);
bool fw_update_transfer_active(void);

void fw_update_read_status(uint16_t regs[FW_UPDATE_IREG_COUNT]);
fw_update_reply_t fw_update_request(uint16_t op, uint32_t arg,
                                    const uint8_t *data, uint16_t len,
                                    uint32_t now);
fw_update_reply_t fw_update_data_reply(void);

#endif /* APP_FW_UPDATE_H */
