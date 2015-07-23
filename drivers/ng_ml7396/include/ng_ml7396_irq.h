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
 * @brief       IRQ thread for ML7396 driver
 *
 */

#ifndef NG_ML7396_IRQ_H_
#define NG_ML7396_IRQ_H_

#include "ng_ml7396.h"

#ifdef __cplusplus
extern "C" {
#endif

kernel_pid_t ng_ml7396_irq_init(ng_ml7396_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NG_ML7396_IRQ_H_ */
