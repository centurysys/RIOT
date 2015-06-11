/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_ng_ml7396
 * @{
 *
 * @file
 * @brief       Internal interfaces for ML7396 driver
 *
 */

#ifndef NG_ML7396_INTERNAL_H_
#define NG_ML7396_INTERNAL_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Write Register(s) */
void ng_ml7396_reg_write(const ng_ml7396_t *dev, uint16_t reg, uint8_t value);
void ng_ml7396_reg_writes(const ng_ml7396_t *dev, uint16_t reg,
                          uint8_t *value, size_t len);

/* Read Register(s) */
uint8_t ng_ml7396_reg_read(const ng_ml7396_t *dev, uint16_t reg);
void ng_ml7396_reg_reads(const ng_ml7396_t *dev, uint16_t reg,
                         uint8_t *buf, size_t len);

/* Access FIFO */
void ng_ml7396_fifo_writes(const ng_ml7396_t *dev, const uint8_t *values,
                           size_t len);
int ng_ml7396_fifo_reads(const ng_ml7396_t *dev, uint8_t *values, size_t len);

/* Manage Interrupts */
uint32_t ng_ml7396_get_interrupt_status(const ng_ml7396_t *dev);
uint32_t ng_ml7396_get_interrupt_enable(const ng_ml7396_t *dev);
void ng_ml7396_clear_interrupts(const ng_ml7396_t *dev, uint32_t interrupts);
void ng_ml7396_set_interrupt_enable(const ng_ml7396_t *dev, uint32_t interrupts);
void ng_ml7396_set_interrupt_mask(const ng_ml7396_t *dev, uint32_t interrupts);

void ng_ml7396_phy_reset(const ng_ml7396_t *dev);

/* Internal use */
int _ng_ml7396_wait_rf_stat_poll(ng_ml7396_t *dev, uint8_t stat);
void _ng_ml7396_clear_rx_interrupts(ng_ml7396_t *dev, int page, int clear_fifo);

#ifdef __cplusplus
}
#endif

#endif /* NG_ML7396_INTERNAL_H_ */
