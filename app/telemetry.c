/*
 * telemetry.c — заполнение входных регистров.
 */

#include "telemetry.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"
#define DEPLOYED_ACTIVE_LOW 1u

static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} DEPLOYED_PINS[] = {
    [0] = { RELEASE1_DEPLOYED_GPIO_Port, RELEASE1_DEPLOYED_Pin },
    [1] = { RELEASE2_DEPLOYED_GPIO_Port, RELEASE2_DEPLOYED_Pin },
};


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
    uint16_t di = 0u;

    for (unsigned i = 0; i < RELEASE_COUNT; i++) {
        const uint8_t high = (HAL_GPIO_ReadPin(DEPLOYED_PINS[i].port,
                                               DEPLOYED_PINS[i].pin) == GPIO_PIN_SET);
        const uint8_t deployed = DEPLOYED_ACTIVE_LOW ? (uint8_t)!high : high;
        if (deployed) {
            di |= (uint16_t)(1u << RELEASE_SPECS[i].deployed_bit);
        }
    }

    regfile_set_input(IREG_DI_BITS, di);
    regfile_set_input(IREG_FLAGS, flags);
}
