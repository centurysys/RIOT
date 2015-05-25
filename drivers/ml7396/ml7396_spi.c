#include <stdio.h>
#include <unistd.h>

#include "vtimer.h"

#include "ml7396_spi.h"
#include "ml7396_settings.h"

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"

#define REG_WR 0x01
#define REG_RD 0x00

#define REG2BANK(x) (((x) & 0xff00) >> 8)
#define REG2ADDR(x) ((x) & 0x00ff)
#define ADDR(x)     ((x) & 0xfe)

static inline void delay(int loops)
{
    int i;

    for (i = 0; i < loops; i++) {
        __asm__ __volatile__ ("nop");
    }
}

static void ml7396_spi_write(uint8_t addr, uint8_t val)
{
    gpio_clear(ML7396_CS);

    spi_transfer_reg(ML7396_SPI, REG_WR | ADDR(addr), val, 0);

    gpio_set(ML7396_CS);
}

static uint8_t ml7396_spi_read(uint8_t addr)
{
    char value;

    gpio_clear(ML7396_CS);

    spi_transfer_reg(ML7396_SPI, REG_RD | ADDR(addr), 0, &value);

    gpio_set(ML7396_CS);

    return (uint8_t) value;
}

static void ml7396_spi_writes(uint8_t addr, const uint8_t *data, int len)
{
    gpio_clear(ML7396_CS);

    spi_transfer_regs(ML7396_SPI, REG_WR | ADDR(addr), (char *) data, 0, len);

    gpio_set(ML7396_CS);
}

static void ml7396_spi_reads(uint8_t addr, uint8_t *data, int len)
{
    gpio_clear(ML7396_CS);

    spi_transfer_regs(ML7396_SPI, REG_RD | ADDR(addr), 0, (char *) data, len);

    gpio_set(ML7396_CS);
}

static void ml7396_bank_sel(uint8_t bank)
{
    ml7396_spi_write(ML7396_REG_BANK_SEL, bank);
}

/* reg */
void ml7396_reg_write(uint16_t reg, uint8_t value)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    ml7396_spi_write(addr, value);

    spi_release(ML7396_SPI);
}

void ml7396_reg_writes(uint16_t reg, uint8_t *value, int len)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    ml7396_spi_writes(addr, value, len);

    spi_release(ML7396_SPI);
}

uint8_t ml7396_reg_read(uint16_t reg)
{
    uint8_t bank, addr, val;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    val = ml7396_spi_read(addr);

    spi_release(ML7396_SPI);

    return val;
}

void ml7396_reg_reads(uint16_t reg, uint8_t *buf, int len)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    ml7396_spi_reads(addr, buf, len);

    spi_release(ML7396_SPI);
}

/* fifo */
void ml7396_fifo_read(uint8_t *data, radio_packet_length_t length)
{
    uint8_t bank, addr;

    bank = REG2BANK(ML7396_REG_RD_RX_FIFO);
    addr = REG2ADDR(ML7396_REG_RD_RX_FIFO);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    ml7396_spi_reads(addr, data, (int) length);

    spi_release(ML7396_SPI);
}

void ml7396_fifo_write(const uint8_t *data, radio_packet_length_t length)
{
    uint8_t bank, addr;
    //unsigned irqstate;

    bank = REG2BANK(ML7396_REG_WR_TX_FIFO);
    addr = REG2ADDR(ML7396_REG_WR_TX_FIFO);

    spi_acquire(ML7396_SPI);

    ml7396_bank_sel(bank);
    ml7396_spi_writes(addr, data, (int) length);

    spi_release(ML7396_SPI);
}

static uint32_t ml7396_get_interrupt_regs(uint16_t reg)
{
    int i;
    uint32_t val = 0;
    uint8_t buf[4];

    ml7396_reg_reads(reg, buf, 4);

    for (i = 0; i < 4; i++) {
        val |= ((uint32_t) buf[i]) << (8 * i);
    }

    return val;
}

/* get interrupt status */
uint32_t ml7396_get_interrupt_status(void)
{
    return ml7396_get_interrupt_regs(ML7396_REG_INT_SOURCE_GRP1);
}

/* get interrupt enable */
uint32_t ml7396_get_interrupt_enable(void)
{
    return ml7396_get_interrupt_regs(ML7396_REG_INT_EN_GRP1);
}

/* set interrupt status */
void ml7396_clear_interrupts(uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ~interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ml7396_reg_writes(ML7396_REG_INT_SOURCE_GRP1, buf, 4);
}

/* set interrupt enable */
void ml7396_set_interrupt_enable(uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ml7396_get_interrupt_enable();

    status |= interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ml7396_reg_writes(ML7396_REG_INT_EN_GRP1, buf, 4);
}

/* set interrupt enable */
void ml7396_set_interrupt_mask(uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ml7396_get_interrupt_enable();

    status &= ~interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ml7396_reg_writes(ML7396_REG_INT_EN_GRP1, buf, 4);
}

void ml7396_phy_reset(void)
{
    ml7396_reg_write(ML7396_REG_RST_SET, (RST3_EN | RST3));

    if (inISR()) {
        delay(1000);
    }
    else {
        usleep(5);
    }
}
