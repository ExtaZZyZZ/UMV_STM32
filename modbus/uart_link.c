/*
 * uart_link.c — привязка слейва Modbus к USART1.
 */

#include "uart_link.h"

#include "main.h"
#include "board_registers.h"

extern UART_HandleTypeDef huart1;

static mb_slave_t slave;

static volatile uint16_t isr_errors;
static uint16_t          isr_errors_seen;



// Передача по прерыванию и прием реализован свой а не HAL_UART_Transmit_IT
static const uint8_t * volatile tx_ptr;
static volatile uint16_t         tx_left;

static void link_transmit(const uint8_t *data, uint16_t len)
{
    if (len == 0u || tx_left != 0u) {
        return;   
    }
    tx_ptr  = data;
    tx_left = len;

    huart1.Instance->CR1 |= USART_CR1_TXEIE;
}

static void link_critical_enter(void) { __disable_irq(); }
static void link_critical_exit(void)  { __enable_irq();  }

static const mb_hw_ops_t LINK_OPS = {
    .uart_transmit  = link_transmit,
    .critical_enter = link_critical_enter,
    .critical_exit  = link_critical_exit,
};

static uint32_t link_now_ms(void) { return HAL_GetTick(); }

/* ---------------------------------------------------------------- интерфейс */

void uart_link_init(void)
{
    mb_slave_init(&slave, &LINK_OPS, link_now_ms, MODBUS_SLAVE_ADDRESS);

    isr_errors      = 0u;
    isr_errors_seen = 0u;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

mb_slave_t *uart_link_slave(void)
{
    return &slave;
}

uint8_t uart_link_is_up(void)
{
    return (uint8_t)(slave.link_ever_valid &&
                     ((HAL_GetTick() - slave.last_valid_ms) < LINK_TIMEOUT_MS));
}

void uart_receive_isr(void) 
{
    const uint32_t isr = huart1.Instance->ISR;

    if (isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
        huart1.Instance->ICR = USART_ICR_PECF | USART_ICR_FECF |
                               USART_ICR_NECF | USART_ICR_ORECF;
        (void)huart1.Instance->RDR;   
        isr_errors++;
        return;
    }

    if (isr & USART_ISR_RXNE_RXFNE) {

        const uint8_t byte = (uint8_t)(huart1.Instance->RDR & 0xFFu);
        mb_slave_on_rx_byte(&slave, byte);
    }

    if ((isr & USART_ISR_TXE_TXFNF) && (huart1.Instance->CR1 & USART_CR1_TXEIE)) {
        if (tx_left != 0u) {
            huart1.Instance->TDR = *tx_ptr;
            tx_ptr++;
            tx_left--;
        }
        if (tx_left == 0u) {
            huart1.Instance->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void uart_link_task(void)
{

    const uint16_t seen_now = isr_errors;
    if (seen_now != isr_errors_seen) {
        slave.frame_err += (uint16_t)(seen_now - isr_errors_seen);
        isr_errors_seen = seen_now;
    }

    if (tx_left != 0u) {
        return;
    }

    modbus_handler_task(&slave);
}
