#include "stm32l4xx.h"
#include "device/rcc.h"

void system_init(void)
{
    rcc_config();
}

int main(void)
{
    system_init();

    return 0;
}