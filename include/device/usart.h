/*
 * USART: Universal asynchronous receiver-transmitter
 */

#define USART_BUF_SIZE  128

/* Define USARTx_ENABLED to have access to FIFO buffers
 * and interrupts
 */
#define USART2_ENABLED

#ifdef USART1_ENABLED
extern struct sw_fifo_t usart1_tx_fifo;
extern char usart1_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart1_rx_fifo;
extern char usart1_rx_buf[USART_BUF_SIZE];
#endif
#ifdef USART2_ENABLED
extern struct sw_fifo_t usart2_tx_fifo;
extern char usart2_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart2_rx_fifo;
extern char usart2_rx_buf[USART_BUF_SIZE];
#endif
#ifdef USART3_ENABLED
extern struct sw_fifo_t usart3_tx_fifo;
extern char usart3_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart3_rx_fifo;
extern char usart3_rx_buf[USART_BUF_SIZE];
#endif
#ifdef USART4_ENABLED
extern struct sw_fifo_t usart4_tx_fifo;
extern char usart4_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart4_rx_fifo;
extern char usart4_rx_buf[USART_BUF_SIZE];
#endif
#ifdef USART5_ENABLED
extern struct sw_fifo_t usart5_tx_fifo;
extern char usart5_tx_buf[USART_BUF_SIZE];
extern struct sw_fifo_t usart5_rx_fifo;
extern char usart5_rx_buf[USART_BUF_SIZE];
#endif

void usart_config(void);
void usart_setup(USART_TypeDef *usart, uint32_t baudrate);
void usart_str_send(USART_TypeDef *usart, char *str);