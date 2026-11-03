

#include "inputs.h"

#include "main.h"
#include "board_registers.h"

#define IN_DEBOUNCE_MS 20u
#define IN_ACTIVE_LOW   1u

static const struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} IN_PINS[] = {
    [IN_RELEASE1_DEPLOYED] = { IS_BUOY_1_GPIO_Port, IS_BUOY_1_Pin },
    [IN_RELEASE2_DEPLOYED] = { IS_BOUY_2_GPIO_Port, IS_BOUY_2_Pin },
};

#define IN_PIN_COUNT (sizeof(IN_PINS) / sizeof(IN_PINS[0]))

_Static_assert(IN_PIN_COUNT == RELEASE_COUNT,
               "One presence input is required per release unit");

static uint8_t  stable_ms[IN_PIN_COUNT];
static uint16_t bits;

static uint8_t in_raw(unsigned i)
{
    const uint8_t high = (HAL_GPIO_ReadPin(IN_PINS[i].port,
                                           IN_PINS[i].pin) == GPIO_PIN_SET);
    return IN_ACTIVE_LOW ? (uint8_t)!high : high;
}

void inputs_init(void)
{
    bits = 0u;
    for (unsigned i = 0; i < IN_PIN_COUNT; i++) {
        stable_ms[i] = 0u;
        bits |= (uint16_t)((uint16_t)in_raw(i) << i);
    }
}

void inputs_step(void)
{
    for (unsigned i = 0; i < IN_PIN_COUNT; i++) {
        if (in_raw(i) == ((bits >> i) & 1u)) {
            stable_ms[i] = 0u;
        } else if (++stable_ms[i] >= IN_DEBOUNCE_MS) {
            bits ^= (uint16_t)(1u << i);
            stable_ms[i] = 0u;
        }
    }
}

uint16_t inputs_bits(void)
{
    return bits;
}
