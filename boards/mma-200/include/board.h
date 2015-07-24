/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_mma-200 MMA-200
 * @ingroup     boards
 * @brief       Board specific files for the MMA-200 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the MMA-200 board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef __BOARD_H
#define __BOARD_H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Define the nominal CPU core clock in this board
 */
#define F_CPU               CLOCK_CORECLOCK

/**
 * @name Assign the hardware timer
 */
#define HW_TIMER            TIMER_0

/**
 * @name Define the interface to the ML7396 radio
 * @{
 */
#define ML7396_SPI       SPI_0
#define ML7396_CS        GPIO(PORT_A, 4)
#define ML7396_INT       GPIO(PORT_A, 2)
#define ML7396_RESET     GPIO(PORT_C, 2)
#define ML7396_TCXO      GPIO(PORT_B, 0)
#define ML7396_SPI_CLK   SPI_SPEED_15MHZ
/** @} */

/**
 * @name Define UART device and baudrate for stdio
 * @{
 */
#define STDIO               UART_0
#define STDIO_BAUDRATE      (115200U)
#define STDIO_RX_BUFSIZE    (64U)
#define STDIO_TX_BUFSIZE    (4096U * 3)
/** @} */

/**
 * @name Define UART device and baudrate for host I/F
 * @{
 */
#define HOSTIF              UART_1
#define HOSTIF_BAUDRATE     (115200U)
#define HOSTIF_RX_BUFSIZE   (64U)
/** @} */

/**
 * @name LED pin definitions
 * @{
 */
#define LED_PORT            GPIOA
#define LD0_PIN             (1 << 15)
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LD0_ON              (LED_PORT->BSRRL = LD0_PIN)
#define LD0_OFF             (LED_PORT->BSRRH = LD0_PIN)
#define LD0_TOGGLE          (LED_PORT->ODR ^= LD0_PIN)

/* for compatability to other boards */
#define LED_GREEN_ON        LD0_ON
#define LED_GREEN_OFF       LD0_OFF
#define LED_GREEN_TOGGLE    LD0_TOGGLE
/** @} */

/**
 * @brief define radio packet length
 */
typedef uint8_t radio_packet_length_t;

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** __BOARD_H */
/** @} */
