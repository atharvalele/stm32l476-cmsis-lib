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
    /* set pin 8 to output mode */
    gpio_mode_set(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE);
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

/* Set GPIO Pin Modes - I/P, O/P, Analog, AF  & Pull Up / Pull Down*/
void gpio_mode_set(GPIO_TypeDef *port, uint16_t pins, uint8_t mode, uint8_t pupd_sett)
{
    uint8_t i;
    uint32_t moder, pupdr;

    /*
     * We only wan't to change the pins that are passed,
     * so keep the original values & mask the passed ones
     */
    moder = port->MODER;
    pupdr = port->PUPDR;

    for (i = 0; i < 16; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        moder &= ~GPIO_SETT_MASK(i);
        moder |= GPIO_SETT(i, mode);

        pupdr &= ~GPIO_SETT_MASK(i);
        pupdr |= GPIO_SETT(i, pupd_sett);
    }

    /* Apply the settings */
    port->MODER = moder;
    port->PUPDR = pupdr;
}

/* Turn ON all GPIO pins passed as an argument */
void gpio_set(GPIO_TypeDef *port, uint16_t pins)
{
    port->BSRR |= pins;
}

/* Turn OFF all GPIO pins passed as an argument */
void gpio_clear(GPIO_TypeDef *port, uint16_t pins)
{
    port->BRR |= pins;
}