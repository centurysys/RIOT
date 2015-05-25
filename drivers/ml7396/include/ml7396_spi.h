/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_ml7396
 * @{
 *
 * @file
 * @brief       Register access function definitions for the ML7396 device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef ML7396_SPI_H_
#define ML7396_SPI_H_

#include <stdint.h>

#include "board.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DEBUG
#define dprintf(fmt, arg...) do {printf(fmt, ##arg);} while (0)
#else
#define dprintf(fmt, arg...) do {} while (0)
#endif

static inline void ml7396_disableIRQ(void)
{
    gpio_irq_disable(ML7396_INT);
}

static inline void ml7396_enableIRQ(void)
{
    gpio_irq_enable(ML7396_INT);
}

void ml7396_reg_write(uint16_t addr, uint8_t value);
uint8_t ml7396_reg_read(uint16_t addr);

void ml7396_reg_writes(uint16_t reg, uint8_t *value, int len);
void ml7396_reg_reads(uint16_t reg, uint8_t *buf, int len);

void ml7396_fifo_read(uint8_t *data, radio_packet_length_t length);
void ml7396_fifo_write(const uint8_t *data, radio_packet_length_t length);

uint32_t ml7396_get_interrupt_status(void);
uint32_t ml7396_get_interrupt_enable(void);
void ml7396_clear_interrupts(uint32_t interrupts);
void ml7396_set_interrupt_enable(uint32_t interrupts);
void ml7396_set_interrupt_mask(uint32_t interrupts);

void ml7396_phy_reset(void);


#ifdef __cplusplus
}
#endif

#endif /* ML7396_SPI_H_ */
/** @} */
