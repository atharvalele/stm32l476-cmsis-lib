/*
 * GPIO: General Purpose Input/Output
 */

#include "stm32l4xx.h"
#include "device/gpio.h"

void gpio_config(void)
{
    /* Initialize status led */

    /* Enable clock */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOEEN);
    /* set pin 2 to output mode */
    MODIFY_REG(STATUS_LED_PORT->MODER, GPIO_MODER_MODE8, GPIO_MODER_MODE8_0);
    /* set it as push-pull */
    MODIFY_REG(STATUS_LED_PORT->OTYPER, GPIO_OTYPER_OT8, 0x00000000U);
    /* set it to max speed */
    MODIFY_REG(STATUS_LED_PORT->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8, GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1);
}

/* Toggle All GPIO pins passed as an argument */
void gpio_toggle(GPIO_TypeDef *port, uint16_t pins)
{
    uint32_t portval;

    portval = port->ODR ;
    port->BSRR = (portval & pins) << 16 | (~portval & pins);
}