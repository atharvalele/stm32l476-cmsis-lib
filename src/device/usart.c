/*
 * USART: Universal asynchronous receiver-transmitter
 */

#include "stm32l4xx.h"
#include "device/usart.h"
#include "utils/sw_fifo.h"

#include <string.h>

#ifdef USART1_ENABLED
struct sw_fifo_t usart1_tx_fifo;
char usart1_tx_buf[USART_BUF_SIZE];
struct sw_fifo_t usart1_rx_fifo;
char usart1_rx_buf[USART_BUF_SIZE];
struct usart_handle_t usart1 = {
    .port = USART1,
    .baudrate = 115200,
    .tx_fifo = &usart1_tx_fifo,
    .tx_buf = usart1_tx_buf,
    .rx_fifo = &usart1_rx_fifo,
    .rx_buf = usart1_rx_buf,
};
#endif
#ifdef USART2_ENABLED
struct sw_fifo_t usart2_tx_fifo;
char usart2_tx_buf[USART_BUF_SIZE];
struct sw_fifo_t usart2_rx_fifo;
char usart2_rx_buf[USART_BUF_SIZE];
struct usart_handle_t usart2 = {
    .port = USART2,
    .baudrate = 115200,
    .tx_fifo = &usart2_tx_fifo,
    .tx_buf = usart2_tx_buf,
    .rx_fifo = &usart2_rx_fifo,
    .rx_buf = usart2_rx_buf,
};
#endif
#ifdef USART3_ENABLED
struct sw_fifo_t usart3_tx_fifo;
char usart3_tx_buf[USART_BUF_SIZE];
struct sw_fifo_t usart3_rx_fifo;
char usart3_rx_buf[USART_BUF_SIZE];
struct usart_handle_t usart3 = {
    .port = USART3,
    .baudrate = 115200,
    .tx_fifo = &usart3_tx_fifo,
    .tx_buf = usart3_tx_buf,
    .rx_fifo = &usart3_rx_fifo,
    .rx_buf = usart3_rx_buf,
};
#endif
#ifdef USART4_ENABLED
struct sw_fifo_t usart4_tx_fifo;
char usart4_tx_buf[USART_BUF_SIZE];
struct sw_fifo_t usart4_rx_fifo;
char usart4_rx_buf[USART_BUF_SIZE];
struct usart_handle_t usart4 = {
    .port = USART4,
    .baudrate = 115200,
    .tx_fifo = &usart4_tx_fifo,
    .tx_buf = usart4_tx_buf,
    .rx_fifo = &usart4_rx_fifo,
    .rx_buf = usart4_rx_buf,
};
#endif
#ifdef USART5_ENABLED
struct sw_fifo_t usart5_tx_fifo;
char usart5_tx_buf[USART_BUF_SIZE];
struct sw_fifo_t usart5_rx_fifo;
char usart5_rx_buf[USART_BUF_SIZE];
struct usart_handle_t usart5 = {
    .port = USART5,
    .baudrate = 115200,
    .tx_fifo = &usart5_tx_fifo,
    .tx_buf = usart5_tx_buf,
    .rx_fifo = &usart5_rx_fifo,
    .rx_buf = usart5_rx_buf,
};
#endif

/* Setup all USARTs */
void usart_config(void)
{
#ifdef USART1_ENABLED
    SET_BIT(RCC->APB2ENR1, RCC_APB2ENR_USART1EN);
    usart_setup(&usart1);
#endif
#ifdef USART2_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN);
    usart_setup(&usart2);
#endif
#ifdef USART3_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART3EN);
    usart_setup(&usart3);
#endif
#ifdef USART4_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART4EN);
    usart_setup(&usart4);
#endif
#ifdef USART5_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART5EN);
    usart_setup(&usart5);
#endif
}

/* Setup a given USART port with its baudrate, 8N1 Async Mode, Tx/Rx Interrupt Enabled */
void usart_setup(struct usart_handle_t *usart)
{
    /* Initialize the FIFO */
    sw_fifo_init(usart->tx_fifo, usart->tx_buf, USART_BUF_SIZE); 
    sw_fifo_init(usart->rx_fifo, usart->rx_buf, USART_BUF_SIZE);

    /* Baudrate calculation:
     * 
     * Baud = f(CK) / USARTDIV
     * USART gets APB clock and we don't divice APB Clock.
     * i.e. APB clock == System Clock
     */
    usart->port->BRR = SystemCoreClock / usart->baudrate;
    
    /* 1 stop bit, autobaud disabled, rx timeout disabled
     * DMA disabled, RTS/CTS disabled, IrDA disabled
     */
    usart->port->CR3 = 0x0000;
    /* 8Nx, tx/rx interrupt enabled, tx/rx enabled, enable USART */
    usart->port->CR1 = 0x00EC;
    /* finally enable the USART */
    usart->port->CR1 |= 0x0001;
    /* Clear interupts */
    usart->port->ICR |= USART_ICR_TCCF;
}

void usart_str_send(struct usart_handle_t *usart, char *str)
{
    /*
     * If the length of what we're sending is greater than the free space
     * in the buffer, wait for the buffer to clear up so that we don't lose
     * any data. The cost in time should be a few milliseconds at most.
     */
    uint16_t len;

    len = strlen(str);
    if (len > 0) {
        if (len >= sw_fifo_get_free_space(usart->tx_fifo)) {
            while (usart->port->CR1 & USART_CR1_TXEIE);
        }
        sw_fifo_write(usart->tx_fifo, str, strlen(str));
        SET_BIT(usart->port->CR1, USART_CR1_TXEIE);
    }
}

/* USART2 IRQ Handler */
void USART2_IRQHandler()
{
    char outgoing_byte;
    char incoming_byte;
    uint16_t bytes;
    /* Check which type of interrupt */

    /* Tx Complete */
    if (USART2->ISR & USART_ISR_TC) {
        USART2->ICR |= USART_ICR_TCCF;
    }
    /* Tx shift register empty */
    if (USART2->ISR & USART_ISR_TXE) {
        if ((USART2->ISR & USART_ISR_TC) == 0)
        {
            /* Check if FIFO has any bytes to send out, else
             * disable Tx interrupt to save CPU time
             */
            bytes = sw_fifo_read(&usart2_tx_fifo, &outgoing_byte, 1);
            if (bytes != 0)
                USART2->TDR = outgoing_byte;
            else
                CLEAR_BIT(USART2->CR1, USART_ISR_TXE);
        }
    }
    /* Rx shift register not empty */
    if (USART2->ISR & USART_ISR_RXNE) {
        incoming_byte = USART2->RDR;
        sw_fifo_write(&usart2_rx_fifo, &incoming_byte, 1);
    }
}