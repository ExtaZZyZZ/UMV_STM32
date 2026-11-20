/*
 * telemetry.c — заполнение входных регистров.
 */

#include "wdg.h"
#include "main.h"
#include "board_registers.h"
#include "inputs.h"
#include "regfile.h"
#include "uart_link.h"
#include "encoder.h"
#include "telemetry.h"

void telemetry_step(void)
{
    const mb_slave_t *s   = uart_link_slave();
    const uint32_t    now = HAL_GetTick();
    
    regfile_set_input(IREG_UPTIME_LO, (uint16_t)(now & 0xFFFFu));
    regfile_set_input(IREG_UPTIME_HI, (uint16_t)(now >> 16));

    
    regfile_set_input(IREG_CRC_ERR,   s->crc_err);
    regfile_set_input(IREG_FRAME_ERR, s->frame_err);

    uint16_t flags = 0u;

    if (uart_link_is_up()) {
        flags |= (uint16_t)(1u << STATUS_FLAG_LINK_OK);
    }
    regfile_set_input(IREG_ENC_ERR, encoder_err_count());
    if(encoder_is_up()) {
        flags |= (uint16_t)(1u << STATUS_FLAG_ENCODER_OK);
    }
    if(wdg_tripped()) {
        flags |= (uint16_t)(1u << STATUS_FLAG_WDT_TRIPPED);
    }
    regfile_set_input(IREG_DI_BITS, inputs_bits());
    regfile_set_input(IREG_FLAGS, flags);
}
