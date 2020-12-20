/*
 * GPIO: General Purpose Input/Output
 */

/* GPIO Pin definitions */
#define GPIO0		(1 << 0)
#define GPIO1		(1 << 1)
#define GPIO2		(1 << 2)
#define GPIO3		(1 << 3)
#define GPIO4		(1 << 4)
#define GPIO5		(1 << 5)
#define GPIO6		(1 << 6)
#define GPIO7		(1 << 7)
#define GPIO8		(1 << 8)
#define GPIO9		(1 << 9)
#define GPIO10		(1 << 10)
#define GPIO11		(1 << 11)
#define GPIO12		(1 << 12)
#define GPIO13		(1 << 13)
#define GPIO14		(1 << 14)
#define GPIO15		(1 << 15)
#define GPIO_ALL	0xFFFF

/* GPIO Register Masks */
#define GPIO_SETT(n, sett)  ((sett) << (2 * (n)))
#define GPIO_SETT_MASK(n)   (0x03 << (2 * (n)))

#define STATUS_LED_PORT GPIOE
#define STATUS_LED_PIN  GPIO8

/* GPIO Port Mode Defines */
#define GPIO_MODE_INPUT     0x00
#define GPIO_MODE_OUTPUT    0x01
#define GPIO_MODE_AF        0x02
#define GPIO_MODE_ANALOG    0x03

/* GPIO Port Pull Up/Down Defines */
#define GPIO_PUPD_NONE      0x00
#define GPIO_PULL_UP        0x01
#define GPIO_PULL_DOWN      0x03

void gpio_config(void);
void gpio_toggle(GPIO_TypeDef *port, uint16_t pins);
void gpio_mode_set(GPIO_TypeDef *port, uint16_t pins, uint8_t mode,
                   uint8_t pupd_sett);
void gpio_set(GPIO_TypeDef *port, uint16_t pins);
void gpio_clear(GPIO_TypeDef *port, uint16_t pins);