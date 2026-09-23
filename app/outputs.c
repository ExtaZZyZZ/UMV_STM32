/*
 * outputs.c — раскладка маски outputs по ногам и зеркалирование в do_actual.
 */

#include "outputs.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"


#define OUT_MASK_ALL ((uint16_t)((1u << OUT_BIT_COUNT) - 1u))
#define SGU_PULSE_MS 300u
#define SGU_GAP_MS   300u

typedef struct {
    GPIO_TypeDef *port;         
    uint16_t      pin;
    uint8_t       active_high;
} out_pin_t;

static struct {
    uint8_t  state;
    uint8_t  pulse_active;
    uint32_t marked_ms;
} sgu;

static const out_pin_t OUT_PINS[] = {
    [OUT_SPOTLIGHT]  = { SPOTLIGHT_GPIO_Port,  SPOTLIGHT_Pin,  1u },
    [OUT_LIGHTBAR]   = { LIGHTBAR_GPIO_Port,   LIGHTBAR_Pin,   1u },
    [OUT_SGU_LIGHT]  = { SGU_LIGHT_GPIO_Port,  SGU_LIGHT_Pin,  1u },
    [OUT_UNDERWATER] = { UNDERWATER_GPIO_Port, UNDERWATER_Pin, 1u },
};

_Static_assert(sizeof(OUT_PINS) / sizeof(OUT_PINS[0]) == OUT_BIT_COUNT,
               "OUT_PINS does not cover every outputs bit: add the GPIO in "
               "CubeMX (label it like the others) and a row to this table");

static uint16_t actual_mask;

static void output_pin_set(const out_pin_t *p, uint8_t on)
{

    const GPIO_PinState level = (on == p->active_high) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(p->port, p->pin, level);
}

void outputs_init(void)
{
    actual_mask = 0u;
    for (unsigned i = 0; i < OUT_BIT_COUNT; i++) {
        output_pin_set(&OUT_PINS[i], 0u);
    }
    sgu.state        = 0u;
    sgu.pulse_active = 0u;
    sgu.marked_ms    = 0u;
}

void outputs_step(void)
{
    const uint16_t mask = regfile_get_holding(HREG_OUTPUTS) & OUT_MASK_ALL;
    const uint32_t now  = HAL_GetTick();
    const uint8_t  want = (uint8_t)((mask >> OUT_SGU_LIGHT) & 1u);

    for (unsigned i = 0; i < OUT_BIT_COUNT; i++) {
        if (i == (unsigned)OUT_SGU_LIGHT) {
            continue;
        }
        output_pin_set(&OUT_PINS[i], (mask >> i) & 1u);
    }

    if (sgu.pulse_active) {
        if ((now - sgu.marked_ms) >= SGU_PULSE_MS) {
            output_pin_set(&OUT_PINS[OUT_SGU_LIGHT], 0u);
            sgu.pulse_active = 0u;
            sgu.marked_ms    = now;
        }
    } else if (want != sgu.state && (now - sgu.marked_ms) >= SGU_GAP_MS) {
        output_pin_set(&OUT_PINS[OUT_SGU_LIGHT], 1u);
        sgu.pulse_active = 1u;
        sgu.marked_ms    = now;
        sgu.state        = want;
    }

    actual_mask = (uint16_t)((mask & ~(1u << OUT_SGU_LIGHT)) |
                             ((uint16_t)sgu.state << OUT_SGU_LIGHT));

    regfile_set_input(IREG_DO_ACTUAL, actual_mask);
}


uint16_t outputs_actual(void)
{
    return actual_mask;
}

uint8_t outputs_sgu_pulse_active(void)
{
    return sgu.pulse_active;
}

