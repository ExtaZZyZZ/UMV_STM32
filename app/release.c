#include "release.h"
#include "main.h"
#include "board_registers.h"
#include "inputs.h"

#define RELEASE_STROKE_MM        150u
#define RELEASE_SPEED_MM_S        5u
#define RELEASE_RETURN_DELAY_MS 5000u
#define RELEASE_DIR_SETTLE_MS     50u
#define RELEASE_EXTEND_LEVEL GPIO_PIN_SET
#define RELEASE_CONFIRM_TIMEOUT_MS 10000u
#define RELEASE_PRESENT_WHEN_CLOSED 1u

#define RELEASE_RETRACT_LEVEL \
    ((RELEASE_EXTEND_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#define RELEASE_TRAVEL_MS \
    ((RELEASE_STROKE_MM * 1000u) / RELEASE_SPEED_MM_S)

static const struct  {
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
} RELEASE_PINS[RELEASE_COUNT] = 
{
    [0] = {
        EN_LOCK_DRIVER_1_GPIO_Port, EN_LOCK_DRIVER_1_Pin,
        DIR_LOCK_DRIVER_1_GPIO_Port, DIR_LOCK_DRIVER_1_Pin
    },
    [1] = {
        EN_LOCK_DRIVER_2_GPIO_Port, EN_LOCK_DRIVER_2_Pin,
        DIR_LOCK_DRIVER_2_GPIO_Port, DIR_LOCK_DRIVER_2_Pin
    },
};
typedef enum {
    RELEASE_STATE_IDLE = 0,
    RELEASE_STATE_EXTENDING,
    RELEASE_STATE_WAIT_RELEASE,
    RELEASE_STATE_RETURN_DELAY,
    RELEASE_STATE_RETRACTING,
    RELEASE_STATE_FAULT
} release_state_t;

typedef struct {
    release_state_t state;
    uint32_t started_ms;
    uint32_t wait_started_ms;
} release_t;


static release_t releases[RELEASE_COUNT];
static uint8_t release_present(unsigned unit);

void release_init(void)
{
   for (unsigned i = 0; i < RELEASE_COUNT; i++) {
        HAL_GPIO_WritePin(RELEASE_PINS[i].en_port, RELEASE_PINS[i].en_pin, GPIO_PIN_RESET);
        releases[i].state = RELEASE_STATE_IDLE;
        releases[i].started_ms = 0u;
        releases[i].wait_started_ms = 0u;
   } 

}

uint16_t release_cmd_start(uint16_t unit) 
{
    if (unit >= RELEASE_COUNT) {
        return ACK_RESULT_BAD_ARG;
    }
    if (releases[unit].state == RELEASE_STATE_FAULT) {
        return ACK_RESULT_REFUSED_SAFETY;
    }
    if (releases[unit].state != RELEASE_STATE_IDLE) {
        return ACK_RESULT_REFUSED_BUSY;
    }
    if (!release_present(unit)){
        return ACK_RESULT_REFUSED_SAFETY;
    }
    HAL_GPIO_WritePin(RELEASE_PINS[unit].en_port, RELEASE_PINS[unit].en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELEASE_PINS[unit].dir_port, RELEASE_PINS[unit].dir_pin, RELEASE_EXTEND_LEVEL);
    releases[unit].started_ms = HAL_GetTick();
    releases[unit].state = RELEASE_STATE_EXTENDING;

    return ACK_RESULT_OK;
}

void release_step(void)
{
    const uint32_t now = HAL_GetTick();

    for (unsigned i = 0; i < RELEASE_COUNT; i++) {
        const uint32_t elapsed = now - releases[i].started_ms;

        if (releases[i].state == RELEASE_STATE_EXTENDING ||
            releases[i].state == RELEASE_STATE_RETRACTING) {
            if (elapsed >= RELEASE_DIR_SETTLE_MS + RELEASE_TRAVEL_MS) {
                HAL_GPIO_WritePin(RELEASE_PINS[i].en_port,
                                  RELEASE_PINS[i].en_pin,
                                  GPIO_PIN_RESET);

                if (releases[i].state == RELEASE_STATE_EXTENDING) {
                    releases[i].state = RELEASE_STATE_WAIT_RELEASE;
                    releases[i].wait_started_ms = now;
                } else {
                    releases[i].state = RELEASE_STATE_IDLE;
                }

                releases[i].started_ms = now;
            } else if (elapsed >= RELEASE_DIR_SETTLE_MS) {
                HAL_GPIO_WritePin(RELEASE_PINS[i].en_port,
                                  RELEASE_PINS[i].en_pin,
                                  GPIO_PIN_SET);
            }

            continue;
        }

        if (releases[i].state == RELEASE_STATE_WAIT_RELEASE) {
            if ((now - releases[i].wait_started_ms) >=
                RELEASE_CONFIRM_TIMEOUT_MS) {
                releases[i].state = RELEASE_STATE_FAULT;
            } else if (!release_present(i)) {
                releases[i].state = RELEASE_STATE_RETURN_DELAY;
                releases[i].started_ms = now;
            }
        } else if (releases[i].state == RELEASE_STATE_RETURN_DELAY) {
            if (release_present(i)) {
                releases[i].state = RELEASE_STATE_WAIT_RELEASE;
            } else if (elapsed >= RELEASE_RETURN_DELAY_MS) {
                HAL_GPIO_WritePin(RELEASE_PINS[i].en_port,
                                  RELEASE_PINS[i].en_pin,
                                  GPIO_PIN_RESET);

                HAL_GPIO_WritePin(RELEASE_PINS[i].dir_port,
                                  RELEASE_PINS[i].dir_pin,
                                  RELEASE_RETRACT_LEVEL);

                releases[i].started_ms = now;
                releases[i].state = RELEASE_STATE_RETRACTING;
            }
        }
    }
}

static uint8_t release_present(unsigned unit) 
{
    const uint8_t closed = (uint8_t)((inputs_bits() >> RELEASE_SPECS[unit].deployed_bit) & 1u);
    return RELEASE_PRESENT_WHEN_CLOSED ? closed : (uint8_t)!closed;
}