/*
 * RCC: Reset and Clock Control
 */

#include "stm32l4xx.h"
#include "device/rcc.h"

volatile uint32_t rcc_ms_ticks = 0;

/* Set MCU to run at 80MHz speed
 * HSI --> PLL ==> 80MHz
 */
void rcc_config(void)
{
    /* Set appropriate flash access latency - RM0351, pg. 100 */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_4WS);
    while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS);

    /* Turn on HSI */
    SET_BIT(RCC->CR, RCC_CR_HSION);
    while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0);

    /* set default HSI trimming value of 16 */
    MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSITRIM, 16 << RCC_ICSCR_HSITRIM_Pos);

    /* set HSI as PLL source */
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLCFGR_PLLSRC_HSI);

    /*
     * Configure the PLL to generate 80MHz.
     * PLL-R divides by 2 by default, so use 0
     * f(VCO) = PLL_Input * PLLN / PLLM
     *(PLL_R) = f(VCO) / PLLR
     */
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLR,
               RCC_PLLCFGR_PLLM_0 | (20 << RCC_PLLCFGR_PLLN_Pos) | 0x00000000U);

    /* use PLLR as PLL output */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN);

    /* Turn on PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0);


    /* Set AHB/APB prescalers */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);

    /* Switch system clock to PLL */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS)) != RCC_CFGR_SWS_PLL);

    SystemCoreClockUpdate();

    /* Enable 1ms ticks */
    SysTick_Config(SystemCoreClock/1000U);
}

void SysTick_Handler(void)
{
    /* Increment counter necessary in delay_ms()*/
    rcc_ms_ticks++;
}

/* Uses the SysTick Timer to generate an accurate time delay */
void delay_ms(uint32_t ms)
{
    uint32_t curr_ticks;

    curr_ticks = rcc_ms_ticks;

    /* Wait */
    while((rcc_ms_ticks - curr_ticks) < ms);
}