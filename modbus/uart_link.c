/*
 * uart_link.c — привязка слейва Modbus к USART1.
 */

#include "uart_link.h"

#include "main.h"
#include "board_registers.h"
#include "encoder.h"
#include "fw_update.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static mb_slave_t slave;

static uint16_t isr_errors_seen;

typedef struct {
    UART_HandleTypeDef *huart;
    const uint8_t* volatile tx_ptr;
    uint16_t volatile tx_left;
    void(*on_rx_byte)(uint8_t);
    volatile uint16_t errors;
}   uart_port_t;

static void port_transmit(uart_port_t *p, const uint8_t *data, uint16_t len)
{
    if (len == 0u || p->tx_left != 0u) {
        return;
    }
    p->tx_ptr = data;
    p->tx_left = len;
    p->huart->Instance->CR1 |= USART_CR1_TXEIE;
}

static void port_isr(uart_port_t *p)
{
    USART_TypeDef *u = p->huart->Instance;
    const uint32_t isr = u->ISR;
    if (isr & ((USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE))) {
    	u->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    	(void)u->RDR;
    	p->errors++;
    	return;
    }
    if (isr & USART_ISR_RXNE_RXFNE) {
    	p->on_rx_byte((uint8_t)(u->RDR & 0xFFu));
    }
    if ((isr & USART_ISR_TXE_TXFNF) && (u->CR1 & USART_CR1_TXEIE)) {
    	if(p->tx_left != 0u) {
    		u->TDR = *p->tx_ptr;
    		p->tx_ptr++;
    		p->tx_left--;
    	}
        if (p->tx_left == 0u) {
        	u->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

static void slave_on_rx_byte(uint8_t byte)
{
	mb_slave_on_rx_byte(&slave, byte);
}

static uart_port_t port1 = {
		.huart = &huart1,
		.on_rx_byte = slave_on_rx_byte,
};
static void link_transmit(const uint8_t *data, uint16_t len)
{
	port_transmit(&port1, data, len);
}
static uart_port_t port2 = {
		.huart = &huart2,
		.on_rx_byte = encoder_on_rx_byte,
};

static void enc_transmit(const uint8_t *data, uint16_t len)
{
	port_transmit(&port2, data, len);
}

static void link_critical_enter(void) { __disable_irq(); }
static void link_critical_exit(void)  { __enable_irq();  }

static const mb_hw_ops_t LINK_OPS = {
    .uart_transmit  = link_transmit,
    .critical_enter = link_critical_enter,
    .critical_exit  = link_critical_exit,
};

static uint32_t link_now_ms(void) { return HAL_GetTick(); }



void uart_link_init(void)
{
    mb_slave_init(&slave, &LINK_OPS, link_now_ms, MODBUS_SLAVE_ADDRESS);

    port1.errors = 0u;
    isr_errors_seen = 0u;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void enc_link_init(void)
{
	encoder_init(enc_transmit, link_now_ms);
	port2.errors = 0u;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void enc_receive_isr(void)
{
	port_isr(&port2);
}
mb_slave_t *uart_link_slave(void)
{
    return &slave;
}

uint8_t uart_link_is_up(void)
{
    return (uint8_t)(!fw_update_transfer_active() && slave.link_ever_valid &&
                     ((HAL_GetTick() - slave.last_valid_ms) < LINK_TIMEOUT_MS));
}

bool uart_link_tx_idle(void)
{
    return port1.tx_left == 0u && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC);
}

void uart_receive_isr(void) 
{
	port_isr(&port1);
}

void uart_link_task(void)
{
    const uint16_t seen_now = port1.errors;
    if (seen_now != isr_errors_seen) {
        slave.frame_err += (uint16_t)(seen_now - isr_errors_seen);
        isr_errors_seen = seen_now;
    }

    if (port1.tx_left != 0u) {
        return;
    }

    modbus_handler_task(&slave);
}
