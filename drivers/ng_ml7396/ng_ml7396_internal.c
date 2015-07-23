/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ng_ml7396
 * @{
 *
 * @file
 * @brief       Implementation of driver internal functions
 *
 *
 * @}
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "periph/spi.h"
#include "periph/gpio.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_registers.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define REG_WR 0x01
#define REG_RD 0x00

#define REG2BANK(x) (((x) & 0xff00) >> 8)
#define REG2ADDR(x) ((x) & 0x00ff)
#define ADDR(x)     ((x) & 0xfe)


static inline void _delay(int loops)
{
    int i;

    for (i = 0; i < loops; i++) {
        __asm__ __volatile__ ("nop");
    }
}

static void _debug_dump_buffer(char *buf, size_t len)
{
#if ENABLE_DEBUG
    int i;

    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            printf("%04x:", i);
        }

        printf(" %02x", buf[i]);

        if ((i % 16) == 15) {
            puts("");
        }
    }

    if ((i % 16) != 0) {
        puts("");
    }
#endif
}

static void _ng_ml7396_spi_write(const ng_ml7396_t *dev, uint8_t addr, uint8_t val)
{
    gpio_clear(dev->cs_pin);

    spi_transfer_reg(dev->spi, REG_WR | ADDR(addr), val, 0);

    gpio_set(dev->cs_pin);
}

static uint8_t _ng_ml7396_spi_read(const ng_ml7396_t *dev, uint8_t addr)
{
    char value;

    gpio_clear(dev->cs_pin);

    spi_transfer_reg(dev->spi, REG_RD | ADDR(addr), 0, &value);

    gpio_set(dev->cs_pin);

    return (uint8_t) value;
}

static void _ng_ml7396_spi_writes(const ng_ml7396_t *dev, uint8_t addr,
                                  const uint8_t *data, size_t len)
{
    gpio_clear(dev->cs_pin);

    spi_transfer_regs(dev->spi, REG_WR | ADDR(addr), (char *) data, 0, len);

    gpio_set(dev->cs_pin);
}

static void _ng_ml7396_spi_reads(const ng_ml7396_t *dev, uint8_t addr,
                                 uint8_t *data, size_t len)
{
    gpio_clear(dev->cs_pin);

    spi_transfer_regs(dev->spi, REG_RD | ADDR(addr), 0, (char *) data, len);

    gpio_set(dev->cs_pin);
}

static void _ng_ml7396_bank_sel(const ng_ml7396_t *dev, uint8_t bank)
{
    _ng_ml7396_spi_write(dev, ML7396_REG_BANK_SEL, bank);
}

static uint32_t _ng_ml7396_get_interrupt_regs(const ng_ml7396_t *dev, uint16_t reg)
{
    int i;
    uint32_t val = 0;
    uint8_t buf[4];

    ng_ml7396_reg_reads(dev, reg, buf, 4);

    for (i = 0; i < 4; i++) {
        val |= ((uint32_t) buf[i]) << (8 * i);
    }

    return val;
}

void ng_ml7396_lock(ng_ml7396_t *dev)
{
    mutex_lock(&dev->mutex);
}

void ng_ml7396_unlock(ng_ml7396_t *dev)
{
    mutex_unlock(&dev->mutex);
}

void ng_ml7396_reg_write(const ng_ml7396_t *dev, uint16_t reg, uint8_t value)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    DEBUG("[W ]: bank: %d, addr: 0x%02x, value: 0x%02x\n",
          (bank & 0x7f), addr >> 1, value);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    _ng_ml7396_spi_write(dev, addr, value);

    spi_release(dev->spi);
}

void ng_ml7396_reg_writes(const ng_ml7396_t *dev, uint16_t reg,
                          uint8_t *value, size_t len)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    DEBUG("[Ws]: bank: %d, addr: 0x%02x, len: %d\n",
          (bank & 0x7f), addr >> 1, len);
    _debug_dump_buffer(value, len);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    _ng_ml7396_spi_writes(dev, addr, value, len);

    spi_release(dev->spi);
}

uint8_t ng_ml7396_reg_read(const ng_ml7396_t *dev, uint16_t reg)
{
    uint8_t bank, addr, val;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    val = _ng_ml7396_spi_read(dev, addr);

    spi_release(dev->spi);

    DEBUG("[R ]: bank: %d, addr: 0x%02x -> value: 0x%02x\n",
          (bank & 0x7f), addr >> 1, val);

    return val;
}

void ng_ml7396_reg_reads(const ng_ml7396_t *dev, uint16_t reg,
                         uint8_t *buf, size_t len)
{
    uint8_t bank, addr;

    bank = REG2BANK(reg);
    addr = REG2ADDR(reg);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    _ng_ml7396_spi_reads(dev, addr, buf, len);

    DEBUG("[Rs]: bank: %d, addr: 0x%02x, len: %d\n",
          (bank & 0x7f), addr >> 1, len);
    _debug_dump_buffer(buf, len);

    spi_release(dev->spi);
}

void ng_ml7396_fifo_writes(const ng_ml7396_t *dev, const uint8_t *values, size_t len)
{
    uint8_t bank, addr;

    bank = REG2BANK(ML7396_REG_WR_TX_FIFO);
    addr = REG2ADDR(ML7396_REG_WR_TX_FIFO);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    _ng_ml7396_spi_writes(dev, addr, values, len);

    spi_release(dev->spi);
}

int ng_ml7396_fifo_reads(const ng_ml7396_t *dev, uint8_t *values, size_t len)
{
    uint8_t bank, addr;

    bank = REG2BANK(ML7396_REG_RD_RX_FIFO);
    addr = REG2ADDR(ML7396_REG_RD_RX_FIFO);

    spi_acquire(dev->spi);

    _ng_ml7396_bank_sel(dev, bank);
    _ng_ml7396_spi_reads(dev, addr, values, len);

    spi_release(dev->spi);
}

uint32_t ng_ml7396_get_interrupt_status(const ng_ml7396_t *dev)
{
    return _ng_ml7396_get_interrupt_regs(dev, ML7396_REG_INT_SOURCE_GRP1);
}

uint32_t ng_ml7396_get_interrupt_enable(const ng_ml7396_t *dev)
{
    return _ng_ml7396_get_interrupt_regs(dev, ML7396_REG_INT_EN_GRP1);
}

void ng_ml7396_clear_interrupts(const ng_ml7396_t *dev, uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ~interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ng_ml7396_reg_writes(dev, ML7396_REG_INT_SOURCE_GRP1, buf, 4);
}

void ng_ml7396_set_interrupt_enable(const ng_ml7396_t *dev, uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ng_ml7396_get_interrupt_enable(dev);

    status |= interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ng_ml7396_reg_writes(dev, ML7396_REG_INT_EN_GRP1, buf, 4);
}

void ng_ml7396_set_interrupt_mask(const ng_ml7396_t *dev, uint32_t interrupts)
{
    uint8_t buf[4];
    uint32_t status;
    int i;

    status = ng_ml7396_get_interrupt_enable(dev);

    status &= ~interrupts;

    for (i = 0; i < 4; i++) {
        buf[i] = (uint8_t) ((status >> (8 * i)) & 0xff);
    }

    ng_ml7396_reg_writes(dev, ML7396_REG_INT_EN_GRP1, buf, 4);
}

void ng_ml7396_phy_reset(const ng_ml7396_t *dev)
{
    ng_ml7396_reg_write(dev, ML7396_REG_RST_SET, (RST3_EN | RST3));

    if (inISR()) {
        _delay(1000);
    }
    else {
        usleep(50);
    }
}

int _ng_ml7396_wait_interrupt(ng_ml7396_t *dev, uint32_t interrupts,
                              int clear, mutex_t *mutex)
{



    return 0;
}

int _ng_ml7396_wait_rf_stat_poll(ng_ml7396_t *dev, uint8_t stat)
{
    int i, res;
    volatile uint8_t reg;
    volatile uint32_t status;

    res = -ETIMEDOUT;

    for (i = 0; i < 10; i++) {
        reg = ng_ml7396_reg_read(dev, ML7396_REG_RF_STATUS);
        status = ng_ml7396_get_interrupt_status(dev);

        if ((status & INT_RFSTAT_CHANGE) &&
            ((reg & 0xf0) == stat)) {
            ng_ml7396_clear_interrupts(dev, INT_RFSTAT_CHANGE);

            res = 0;
            break;
        }
        usleep(1 * 1000);
    }

    return res;
}

void _ng_ml7396_clear_rx_interrupts(ng_ml7396_t *dev, int page, int clear_fifo)
{
    uint32_t interrupts = 0, status;

    interrupts = INT_SFD_DETECT | INT_RFSTAT_CHANGE;

    status = ng_ml7396_get_interrupt_status(dev);

    if (status & INT_RXFIFO_ERR) {
        puts("! RXFIFO_ERR -> clear_fifo");
        interrupts |= INT_RXFIFO_ERR;
    }

    if (page == 0) {
        interrupts |= (INT_RXFIFO0_DONE | INT_RXFIFO0_CRCERR);
        if ((clear_fifo == 1) && (status & INT_FIFO_CLR0)) {
            //interrupts |= INT_FIFO_CLR0;
        }
    }
    else {
        interrupts |= (INT_RXFIFO1_DONE | INT_RXFIFO1_CRCERR);
        if ((clear_fifo == 1) && (status & INT_FIFO_CLR1)) {
            //interrupts |= INT_FIFO_CLR1;
        }
    }

    if (clear_fifo == 1) {
        printf("*** clear with: 0x%08x (status: 0x%08x)\n",
               (unsigned int) interrupts, (unsigned int) status);
        ng_ml7396_phy_reset(dev);
    }

    ng_ml7396_clear_interrupts(dev, interrupts);
    ng_ml7396_reg_write(dev, ML7396_REG_INT_PD_DATA_IND, 0x00);

    if (clear_fifo == 1) {
        status = ng_ml7396_get_interrupt_status(dev);
        printf(" --> after (status: 0x%08x)\n", (unsigned int) status);
    }
}
