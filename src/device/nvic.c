/*
 * NVIC: Nested Vector Interrupt Controller
 */
#include "stm32l4xx.h"
#include "device/nvic.h"
#include "device/usart.h"

void nvic_config(void)
{
#ifdef USART1_ENABLED
    NVIC_EnableIRQ(USART1_IRQn);
#endif
#ifdef USART2_ENABLED
    NVIC_EnableIRQ(USART2_IRQn);
#endif
#ifdef USART3_ENABLED
    NVIC_EnableIRQ(USART3_IRQn);
#endif
#ifdef USART4_ENABLED
    NVIC_EnableIRQ(UART4_IRQn);
#endif
#ifdef USART5_ENABLED
    NVIC_EnableIRQ(UART5_IRQn);
#endif
}