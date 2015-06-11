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
 * @brief       Netdev interface to ML7396 driver
 *
 */

#ifndef NG_ML7396_NETDEV_H_
#define NG_ML7396_NETDEV_H_

#include "net/ng_netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const ng_netdev_driver_t ng_ml7396_driver;

#ifdef __cplusplus
}
#endif

#endif /* NG_ML7396_NETDEV_H_ */
/** @} */
