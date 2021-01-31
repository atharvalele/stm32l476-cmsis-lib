/*
 * I2C : Inter-Intergrated Circuit
 */

#include "stm32l4xx.h"
#include "device/i2c.h"

#include "utils/sw_fifo.h"

/* I2C declarations */
#ifdef I2C1_ENABLED
struct sw_fifo_t i2c1_tx_fifo;
char i2c1_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c1_rx_fifo;
char i2c1_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c1 = {
    .port = I2C1,
    .tx_fifo = &i2c1_tx_fifo,
    .tx_buf = i2c1_tx_buf,
    .rx_fifo = &i2c1_rx_fifo,
    .rx_buf = i2c1_rx_buf,
};
#endif
#ifdef I2C2_ENABLED
struct sw_fifo_t i2c2_tx_fifo;
char i2c2_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c2_rx_fifo;
char i2c2_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c2 = {
    .port = I2C2,
    .tx_fifo = &i2c2_tx_fifo,
    .tx_buf = i2c2_tx_buf,
    .rx_fifo = &i2c2_rx_fifo,
    .rx_buf = i2c2_rx_buf,
};
#endif
#ifdef I2C3_ENABLED
struct sw_fifo_t i2c3_tx_fifo;
char i2c3_tx_buf[I2C_BUF_SIZE];
struct sw_fifo_t i2c3_rx_fifo;
char i2c3_rx_buf[I2C_BUF_SIZE];
struct i2c_handle_t i2c3 = {
    .port = I2C3,
    .tx_fifo = &i2c3_tx_fifo,
    .tx_buf = i2c3_tx_buf,
    .rx_fifo = &i2c3_rx_fifo,
    .rx_buf = i2c3_rx_buf,
};
#endif


/*
 * Setup all enabled I2C ports.
 * I2C clock is derived from the PCLK domain clock.
 */
void i2c_config(void)
{
#ifdef I2C1_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN);
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL, 0 << RCC_CCIPR_I2C1SEL_Pos);
    i2c_setup(&i2c1);
#endif
#ifdef I2C2_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN);
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C2SEL, 0 << RCC_CCIPR_I2C2SEL_Pos);
    i2c_setup(&i2c2);
#endif
#ifdef I2C3_ENABLED
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN);
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C3SEL, 0 << RCC_CCIPR_I2C3SEL_Pos);
    i2c_setup(&i2c3);
#endif
}

void i2c_setup(struct i2c_handle_t *i2c)
{
    /* Initialize the FIFO */
    sw_fifo_init(i2c->tx_fifo, i2c->tx_buf, I2C_BUF_SIZE);
    sw_fifo_init(i2c->rx_fifo, i2c->rx_buf, I2C_BUF_SIZE);

    /* Disable the I2C perpiheral */
    CLEAR_BIT(i2c->port->CR1, I2C_CR1_PE);

    /*
     * Write the timing register for I2C fast mode, as generated
     * by STM32CubeMX
     */
    WRITE_REG(i2c->port->TIMINGR, 0x00702991);

    /*
     * Enable Rx Interrupt
     * Tx interrupt will be enabled by the transfer function
     */
    SET_BIT(i2c->port->CR1, I2C_CR1_RXIE);

    /* Enable the I2C peripheral */
    SET_BIT(i2c->port->CR1, I2C_CR1_PE);

}

void i2c_start(struct i2c_handle_t *i2c, uint8_t rdwr)
{
    MODIFY_REG(i2c->port->CR2, I2C_CR2_RD_WRN, rdwr << I2C_CR2_RD_WRN_Pos);
    SET_BIT(i2c->port->CR2, I2C_CR2_START);
}

void i2c_stop(struct i2c_handle_t *i2c)
{
    SET_BIT(i2c->port->CR2, I2C_CR2_STOP);
}

void i2c_nack(struct i2c_handle_t *i2c)
{
    SET_BIT(i2c->port->CR2, I2C_CR2_NACK);
}

void i2c_set_address(struct i2c_handle_t *i2c, uint8_t addr)
{
    MODIFY_REG(i2c->port->CR2, I2C_CR2_SADD, addr << 1);
}

void i2c_transfer(struct i2c_handle_t *i2c, uint8_t num_bytes, uint8_t i2c_mode,
                   char reg, char *data)
{
    /* Set the transfer mode */
    i2c->mode = i2c_mode;

    /* Where are we reading/writing from? */
    sw_fifo_write(i2c->tx_fifo, &reg, 1);

    if (i2c->mode == I2C_WRITE) {
        sw_fifo_write(i2c->tx_fifo, data, num_bytes);
        /* Adding 1 to num_bytes to account for reg byte */
        MODIFY_REG(i2c->port->CR2, I2C_CR2_NBYTES, (num_bytes+1) << I2C_CR2_NBYTES_Pos);
    } else if (i2c->mode == I2C_READ) {
        /* note down how many bytes we're reading */
        i2c->num_rx_bytes = num_bytes;
        /* just write which register we're reading from */
        MODIFY_REG(i2c->port->CR2, I2C_CR2_NBYTES, 1 << I2C_CR2_NBYTES_Pos);
    }

    /* Start transmission */
    i2c_start(i2c, I2C_WRITE);

    /* Enable TX/TC interrupts */
    SET_BIT(i2c->port->CR1, I2C_CR1_TXIE);
    SET_BIT(i2c->port->CR1, I2C_CR1_TCIE);
}

/* I2C1 Event IRQ Handler */
#ifdef I2C1_ENABLED
void I2C1_EV_IRQHandler(void)
{
    char outgoing_byte;
    char incoming_byte;
    uint8_t bytes;

    /* Did the peripheral NACK? End tx */
    if (I2C1->ISR & I2C_ISR_NACKF) {
        /* Ack the interrupt*/
        I2C1->ICR |= I2C_ICR_NACKCF;
        i2c_stop(&i2c1);
        /* Disable tx interrupts */
        CLEAR_BIT(I2C1->CR1, I2C_CR1_TXIE);
    }
    
    /* Tx interrupt */
    if (I2C1->ISR & I2C_ISR_TXIS) {
        /* Do we have data to send? */
        bytes = sw_fifo_read(&i2c1_tx_fifo, &outgoing_byte, 1);
        if (bytes != 0)
            I2C1->TXDR = outgoing_byte;
        else
            MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES, 0 << I2C_CR2_NBYTES_Pos);
    }

    /* Tx Complete */
    if (I2C1->ISR & I2C_ISR_TC) {
        /* Read Mode */
        if (I2C1->CR2 & I2C_CR2_RD_WRN) {
            CLEAR_BIT(I2C1->CR1, I2C_CR1_TCIE);
            i2c_stop(&i2c1);
        } else {
            /* Write Mode */
            /* Done with sending data, disable tx interrupts */
            CLEAR_BIT(I2C1->CR1, I2C_CR1_TXIE);
            /* Are we expecting data from the peripheral? */
            if (i2c1.mode == I2C_READ) {
                /* Start with READ */
                MODIFY_REG(I2C1->CR2, I2C_CR2_NBYTES, i2c1.num_rx_bytes << I2C_CR2_NBYTES_Pos);
                i2c_start(&i2c1, I2C_READ);
            } else {
                CLEAR_BIT(I2C1->CR1, I2C_CR1_TCIE);
                i2c_stop(&i2c1);
            }
        }
    }

    /* Rx interrupt */
    if (I2C1->ISR & I2C_ISR_RXNE) {
        incoming_byte = I2C1->RXDR;
        sw_fifo_write(&i2c1_rx_fifo, &incoming_byte, 1);
    }
}
#endif


/* I2C2 Event IRQ Handler */
#ifdef I2C2_ENABLED
void I2C2_EV_IRQHandler(void)
{
    char outgoing_byte;
    char incoming_byte;
    uint8_t bytes;

    /* Did the peripheral NACK? End tx */
    if (I2C2->ISR & I2C_ISR_NACKF) {
        /* Ack the interrupt*/
        I2C2->ICR |= I2C_ICR_NACKCF;
        i2c_stop(&i2c2);
        /* Disable tx interrupts */
        CLEAR_BIT(I2C2->CR2, I2C_CR2_TXIE);
    }
    
    /* Tx interrupt */
    if (I2C2->ISR & I2C_ISR_TXIS) {
        /* Do we have data to send? */
        bytes = sw_fifo_read(&i2c2_tx_fifo, &outgoing_byte, 1);
        if (bytes != 0)
            I2C2->TXDR = outgoing_byte;
        else
            MODIFY_REG(I2C2->CR2, I2C_CR2_NBYTES, 0 << I2C_CR2_NBYTES_Pos);
    }

    /* Tx Complete */
    if (I2C2->ISR & I2C_ISR_TC) {
        /* Read Mode */
        if (I2C2->CR2 & I2C_CR2_RD_WRN) {
            CLEAR_BIT(I2C2->CR2, I2C_CR2_TCIE);
            i2c_stop(&i2c2);
        } else {
            /* Write Mode */
            /* Done with sending data, disable tx interrupts */
            CLEAR_BIT(I2C2->CR2, I2C_CR2_TXIE);
            /* Are we expecting data from the peripheral? */
            if (i2c2.mode == I2C_READ) {
                /* Start with READ */
                MODIFY_REG(I2C2->CR2, I2C_CR2_NBYTES, i2c2.num_rx_bytes << I2C_CR2_NBYTES_Pos);
                i2c_start(&i2c2, I2C_READ);
            } else {
                CLEAR_BIT(I2C2->CR2, I2C_CR2_TCIE);
                i2c_stop(&i2c2);
            }
        }
    }

    /* Rx interrupt */
    if (I2C2->ISR & I2C_ISR_RXNE) {
        incoming_byte = I2C2->RXDR;
        sw_fifo_write(&i2c2_rx_fifo, &incoming_byte, 1);
    }
}
#endif

/* I2C3 Event IRQ Handler */
#ifdef I2C3_ENABLED
void I2C3_EV_IRQHandler(void)
{
    char outgoing_byte;
    char incoming_byte;
    uint8_t bytes;

    /* Did the peripheral NACK? End tx */
    if (I2C3->ISR & I2C_ISR_NACKF) {
        /* Ack the interrupt*/
        I2C3->ICR |= I2C_ICR_NACKCF;
        i2c_stop(&i2c3);
        /* Disable tx interrupts */
        CLEAR_BIT(I2C3->CR2, I2C_CR2_TXIE);
    }
    
    /* Tx interrupt */
    if (I2C3->ISR & I2C_ISR_TXIS) {
        /* Do we have data to send? */
        bytes = sw_fifo_read(&i2c3_tx_fifo, &outgoing_byte, 1);
        if (bytes != 0)
            I2C3->TXDR = outgoing_byte;
        else
            MODIFY_REG(I2C3->CR2, I2C_CR2_NBYTES, 0 << I2C_CR2_NBYTES_Pos);
    }

    /* Tx Complete */
    if (I2C3->ISR & I2C_ISR_TC) {
        /* Read Mode */
        if (I2C3->CR2 & I2C_CR2_RD_WRN) {
            CLEAR_BIT(I2C3->CR2, I2C_CR2_TCIE);
            i2c_stop(&i2c3);
        } else {
            /* Write Mode */
            /* Done with sending data, disable tx interrupts */
            CLEAR_BIT(I2C3->CR2, I2C_CR2_TXIE);
            /* Are we expecting data from the peripheral? */
            if (i2c3.mode == I2C_READ) {
                /* Start with READ */
                MODIFY_REG(I2C3->CR2, I2C_CR2_NBYTES, i2c3.num_rx_bytes << I2C_CR2_NBYTES_Pos);
                i2c_start(&i2c3, I2C_READ);
            } else {
                CLEAR_BIT(I2C3->CR2, I2C_CR2_TCIE);
                i2c_stop(&i2c3);
            }
        }
    }

    /* Rx interrupt */
    if (I2C3->ISR & I2C_ISR_RXNE) {
        incoming_byte = I2C3->RXDR;
        sw_fifo_write(&i2c3_rx_fifo, &incoming_byte, 1);
    }
}
#endif