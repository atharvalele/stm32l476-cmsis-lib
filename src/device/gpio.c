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
    /* set it as push-pull & max speed output */
    gpio_output_options_set(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_OUTPUT_PUSHPULL, GPIO_OUTPUT_LOW_SPEED);

    /* Setup GPIO for USART */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIODEN);
    gpio_mode_set(GPIOD, (GPIO5 | GPIO6), GPIO_MODE_AF, GPIO_PUPD_NONE);
    gpio_output_options_set(GPIOD, GPIO5, GPIO_OUTPUT_PUSHPULL, GPIO_OUTPUT_VERYHIGH_SPEED);
    gpio_af_set(GPIOD, (GPIO5 | GPIO6), GPIO_AF7);
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

/* Set GPIO Output Type & Speed */
void gpio_output_options_set(GPIO_TypeDef *port, uint16_t pins, uint8_t otype_sett,
                             uint8_t ospeed_sett)
{
    uint8_t i;
    uint32_t ospeedr;

    /*
     * We only wan't to change the pins that are passed,
     * so keep the original values & mask the passed ones
     */
    ospeedr = port->OSPEEDR;

    if (otype_sett == GPIO_OUTPUT_PUSHPULL) {
        port->OTYPER &= ~pins;
    } else {
        port->OTYPER |= pins;
    }

    for (i = 0; i < 16; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        ospeedr &= ~GPIO_SETT_MASK(i);
        ospeedr |= GPIO_SETT(i, ospeed_sett);
    }

    /* Apply the settings */
    port->OSPEEDR = ospeedr;
}

/* Set all passed pins of a port as the alternate function selected*/
void gpio_af_set(GPIO_TypeDef *port, uint16_t pins, uint8_t gpio_af)
{
    uint8_t i;
    uint32_t portval;

    /* Set the AFs of pins 0-7 */
    portval = port->AFR[0];
    for (i = 0; i < 8; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        portval &= ~GPIO_AF_SETT_MASK(i);
        portval |= GPIO_AF_SETT(i, gpio_af);
    }
    port->AFR[0] = portval;

    /* Set the AFs of pins 8-15 */
    portval = port->AFR[1];
    for (i = 8; i < 16; i++) {
        if (((1 << i) & pins) == 0) {
            continue;
        }

        portval &= ~GPIO_AF_SETT_MASK(i);
        portval |= GPIO_AF_SETT(i, gpio_af);
    }
    port->AFR[1] = portval;
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