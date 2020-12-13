/*
 * RCC: Reset and Clock Control
 */

extern volatile uint8_t secflag;
extern volatile uint16_t ms_ticks;

void rcc_config(void);
void delay_ms(uint32_t ms);