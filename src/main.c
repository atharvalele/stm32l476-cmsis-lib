#include "stm32l4xx.h"
#include "device/gpio.h"
#include "device/rcc.h"

void system_init(void)
{
    rcc_config();
    gpio_config();
}

int main(void)
{
    system_init();

    while (1) {
        if (secflag) {
            gpio_toggle(STATUS_LED_PORT, STATUS_LED_PIN);
            secflag = 0;
        }
    }

    return 0;
}