/*
 * telemetry.c — заполнение входных регистров.
 */

#include "telemetry.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"

void telemetry_step(void)
{
    const mb_slave_t *s   = uart_link_slave();
    const uint32_t    now = HAL_GetTick();
    
    regfile_set_input(IREG_UPTIME_LO, (uint16_t)(now & 0xFFFFu));
    regfile_set_input(IREG_UPTIME_HI, (uint16_t)(now >> 16));

    
    regfile_set_input(IREG_CRC_ERR,   s->crc_err);
    regfile_set_input(IREG_FRAME_ERR, s->frame_err);

    uint16_t flags = 0u;

    if (s->link_ever_valid && (now - s->last_valid_ms) < LINK_TIMEOUT_MS) {
        flags |= (uint16_t)(1u << STATUS_FLAG_LINK_OK);
    }

    regfile_set_input(IREG_FLAGS, flags);
}
