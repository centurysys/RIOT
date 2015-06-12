/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   board_mma-200
 * @{
 *
 * @file
 * @brief     ml7396 board specific configuration
 *
 */

#ifndef NG_ML7396_PARAMS_H
#define NG_ML7396_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name ML7396 configuration
 */
static const  ml7396_params_t ml7396_params[] =
    {
        {
            .spi = ML7396_SPI,
            .spi_speed = ML7396_SPI_CLK,
            .cs_pin = ML7396_CS,
            .int_pin = ML7396_INT,
            .reset_pin = ML7396_RESET,
            .tcxo_pin = ML7396_TCXO,
        },
    };
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* NG_ML7396_PARAMS_H */
/** @} */
