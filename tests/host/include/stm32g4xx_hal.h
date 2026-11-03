#ifndef HOST_STM32G4XX_HAL_H
#define HOST_STM32G4XX_HAL_H

#include <stdint.h>

typedef struct {
    uint16_t IDR;
    uint16_t ODR;
} GPIO_TypeDef;

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum { HAL_OK = 0 } HAL_StatusTypeDef;
typedef struct { unsigned unused; } UART_HandleTypeDef;

extern GPIO_TypeDef host_gpio_a, host_gpio_b, host_gpio_c;
extern GPIO_TypeDef host_gpio_d, host_gpio_e, host_gpio_f;

#define GPIOA (&host_gpio_a)
#define GPIOB (&host_gpio_b)
#define GPIOC (&host_gpio_c)
#define GPIOD (&host_gpio_d)
#define GPIOE (&host_gpio_e)
#define GPIOF (&host_gpio_f)

#define GPIO_PIN_0  ((uint16_t)0x0001u)
#define GPIO_PIN_1  ((uint16_t)0x0002u)
#define GPIO_PIN_2  ((uint16_t)0x0004u)
#define GPIO_PIN_3  ((uint16_t)0x0008u)
#define GPIO_PIN_4  ((uint16_t)0x0010u)
#define GPIO_PIN_5  ((uint16_t)0x0020u)
#define GPIO_PIN_6  ((uint16_t)0x0040u)
#define GPIO_PIN_7  ((uint16_t)0x0080u)
#define GPIO_PIN_8  ((uint16_t)0x0100u)
#define GPIO_PIN_9  ((uint16_t)0x0200u)
#define GPIO_PIN_10 ((uint16_t)0x0400u)
#define GPIO_PIN_11 ((uint16_t)0x0800u)
#define GPIO_PIN_12 ((uint16_t)0x1000u)
#define GPIO_PIN_13 ((uint16_t)0x2000u)
#define GPIO_PIN_14 ((uint16_t)0x4000u)
#define GPIO_PIN_15 ((uint16_t)0x8000u)

uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
                                    const uint8_t *data, uint16_t length,
                                    uint32_t timeout);

#endif
