/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_ng_ml7396 ML7396 driver
 * @ingroup     drivers
 *
 * This module contains drivers for radio devices in Latis's ML7396B.
 * The driver is aimed to work with all devices of this series.
 *
 * @{
 *
 * @file
 * @brief       Interface definition for ML7396 based driver
 *
 */

#ifndef NG_ML7396_H_
#define NG_ML7396_H_

#include <stdint.h>

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/ng_netdev.h"
#include "ng_ml7396.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum possible packet size in byte
 */
#define NG_ML7396_MAX_PKT_LENGTH     (255)

/**
 * @brief   Default addresses used if the CPUID module is not present
 * @{
 */
#define NG_ML7396_DEFAULT_ADDR_SHORT (0x0230)
#define NG_ML7396_DEFAULT_ADDR_LONG  (0x1222334455667788)
/** @} */

/**
  * @brief   Channel configuration
  * @{
  */
#ifdef MODULE_NG_AT86RF212B
/* the AT86RF212B has a sub-1GHz radio */
#define NG_ML7396_MIN_CHANNEL        (0)
#define NG_ML7396_MAX_CHANNEL        (10)
#define NG_ML7396_DEFAULT_CHANNEL    (5)
#else
#define NG_ML7396_MIN_CHANNEL        (11U)
#define NG_ML7396_MAX_CHANNEL        (26U)
#define NG_ML7396_DEFAULT_CHANNEL    (17U)
#endif
/** @} */

/**
  * @brief   Frequency configuration
  * @{
  */
#ifdef MODULE_NG_AT86RF212B
typedef enum {
    NG_ML7396_FREQ_915MHZ,    /**< frequency 915MHz enabled */
    NG_ML7396_FREQ_868MHZ,    /**< frequency 868MHz enabled */
} ng_ml7396_freq_t;
#endif
/** @} */

/**
 * @brief   Default PAN ID
 *
 * TODO: Read some global network stack specific configuration value
 */
#define NG_ML7396_DEFAULT_PANID      (0x0023)

/**
 * @brief   Default TX power (0dBm)
 */
#define NG_ML7396_DEFAULT_TXPOWER    (0U)

/**
 * @brief   Device descriptor for ML7396 radio devices
 */
typedef struct {
    /* netdev fields */
    const ng_netdev_driver_t *driver;   /**< pointer to the devices interface */
    ng_netdev_event_cb_t event_cb;      /**< netdev event callback */
    kernel_pid_t mac_pid;               /**< the driver's thread's PID */
    kernel_pid_t tx_pid;                /**< the driver's TX thread's PID */

    /* device specific fields */
    spi_t spi;                          /**< used SPI device */
    gpio_t cs_pin;                      /**< chip select pin */
    gpio_t reset_pin;                   /**< reset pin */
    gpio_t int_pin;                     /**< external interrupt pin */
    gpio_t tcxo_pin;                    /**< TCXO control pin */
    ng_nettype_t proto;                 /**< protocol the radio expects */
    uint8_t state;                      /**< current state of the radio */
    uint8_t seq_nr;                     /**< sequence number to use next */
    uint8_t frame_len;                  /**< length of the current TX frame */
    uint16_t pan;                       /**< currently used PAN ID */
    uint8_t chan;                       /**< currently used channel */
    uint8_t addr_short[2];              /**< the radio's short address */
    uint8_t addr_long[8];               /**< the radio's long address */
    uint16_t options;                   /**< state of used options */
    uint8_t idle_state;                 /**< state to return to after sending */

    mutex_t mutex;

    uint8_t max_retries;
} ng_ml7396_t;


/**
 * @brief   Initialize a given Ml7396 device
 *
 * @param[out] dev          device descriptor
 * @param[in] spi           SPI bus the device is connected to
 * @param[in] spi_speed     SPI speed to use
 * @param[in] cs_pin        GPIO pin connected to chip select
 * @param[in] int_pin       GPIO pin connected to the interrupt pin
 * @param[in] sleep_pin     GPIO pin connected to the sleep pin
 * @param[in] reset_pin     GPIO pin connected to the reset pin
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ng_ml7396_init(ng_ml7396_t *dev, spi_t spi, spi_speed_t spi_speed,
                   gpio_t cs_pin, gpio_t int_pin, gpio_t reset_pin);

/**
 * @brief struct holding all params needed for device initialization
 */
typedef struct ml7396_params {
    spi_t spi;              /**< SPI bus the device is connected to */
    spi_speed_t spi_speed;  /**< SPI speed to use */
    gpio_t cs_pin;          /**< GPIO pin connected to chip select */
    gpio_t int_pin;         /**< GPIO pin connected to the interrupt pin */
    gpio_t reset_pin;       /**< GPIO pin connected to the reset pin */
} ml7396_params_t;

/**
 * @brief   Trigger a hardware reset and configure radio with default values
 *
 * @param[in] dev           device to reset
 */
void ng_ml7396_reset(ng_ml7396_t *dev);

/**
 * @brief   Trigger a clear channel assessment
 *
 * @param[in] dev           device to use
 *
 * @return                  true if channel is clear
 * @return                  false if channel is busy
 */
bool ng_ml7396_cca(ng_ml7396_t *dev);

/**
 * @brief   Get the short address of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set (2-byte) short address
 */
uint16_t ng_ml7396_get_addr_short(ng_ml7396_t *dev);

/**
 * @brief   Set the short address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr          (2-byte) short address to set
 */
void ng_ml7396_set_addr_short(ng_ml7396_t *dev, uint16_t addr);

/**
 * @brief   Get the configured long address of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set (8-byte) long address
 */
uint64_t ng_ml7396_get_addr_long(ng_ml7396_t *dev);

/**
 * @brief   Set the long address of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] addr          (8-byte) long address to set
 */
void ng_ml7396_set_addr_long(ng_ml7396_t *dev, uint64_t addr);

/**
 * @brief   Get the configured channel of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set channel
 */
uint8_t ng_ml7396_get_chan(ng_ml7396_t *dev);

/**
 * @brief   Set the channel of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] chan          channel to set
 */
void ng_ml7396_set_chan(ng_ml7396_t *dev, uint8_t chan);

/**
 * @brief   Get the configured PAN ID of the given device
 *
 * @param[in] dev           device to read from
 *
 * @return                  the currently set PAN ID
 */
uint16_t ng_ml7396_get_pan(ng_ml7396_t *dev);

/**
 * @brief   Set the PAN ID of the given device
 *
 * @param[in] dev           device to write to
 * @param[in] pan           PAN ID to set
 */
void ng_ml7396_set_pan(ng_ml7396_t *dev, uint16_t pan);

/**
 * @brief   Get the maximum number of retransmissions
 *
 * @param[in] dev           device to read from
 *
 * @return                  configured number of retransmissions
 */
uint8_t ng_ml7396_get_max_retries(ng_ml7396_t *dev);

/**
 * @brief   Set the maximum number of retransmissions
 *
 * This setting specifies the number of attempts to retransmit a frame, when it
 * was not acknowledged by the recipient, before the transaction gets cancelled.
 * The maximum value is 7.
 *
 * @param[in] dev           device to write to
 * @param[in] max           the maximum number of retransmissions
 */
void ng_ml7396_set_max_retries(ng_ml7396_t *dev, uint8_t max);

/**
 * @brief   Enable or disable driver specific options
 *
 * @param[in] dev           device to set/clear option flag for
 * @param[in] option        option to enable/disable
 * @param[in] state         true for enable, false for disable
 */
void ng_ml7396_set_option(ng_ml7396_t *dev, uint16_t option, bool state);

/**
 * @brief   Get the given devices current internal state
 *
 * @param[in] dev           device to get state of
 * @return                  the current state of the given device
 */
uint8_t ng_ml7396_get_state(ng_ml7396_t *dev);

/**
 * @brief   Set the state of the given device (trigger a state change)
 *
 * @param[in] dev           device to change state of
 * @param[in] state         the targeted new state
 */
void ng_ml7396_set_state(ng_ml7396_t *dev, uint8_t state);

/**
 * @brief   Convenience function for simply sending data
 *
 * @note This function ignores the PRELOADING option
 *
 * @param[in] dev           device to use for sending
 * @param[in] data          data to send (must include IEEE802.15.4 header)
 * @param[in] len           length of @p data
 *
 * @return                  number of bytes that were actually send
 * @return                  0 on error
 */
size_t ng_ml7396_send(ng_ml7396_t *dev, uint8_t *data, size_t len);

/**
 * @brief   Prepare for sending of data
 *
 * This function puts the given device into the TX state, so no receiving of
 * data is possible after it was called.
 *
 * @param[in] dev            device to prepare for sending
 */
void ng_ml7396_tx_prepare(ng_ml7396_t *dev);

/**
 * @brief   Load chunks of data into the transmit buffer of the given device
 *
 * @param[in] dev           device to write data to
 * @param[in] data          buffer containing the data to load
 * @param[in] len           number of bytes in @p buffer
 * @param[in] offset        offset used when writing data to internal buffer
 *
 * @return                  offset + number of bytes written
 */
size_t ng_ml7396_tx_load(ng_ml7396_t *dev, uint8_t *data, size_t len,
                         size_t offset);

/**
 * @brief   Trigger sending of data previously loaded into transmit buffer
 *
 * @param[in] dev           device to trigger
 */
void ng_ml7396_tx_exec(ng_ml7396_t *dev);

/**
 * @brief   Read the length of a received packet
 *
 * @param dev               device to read from
 *
 * @return                  overall length of a received packet in byte
 */
size_t ng_ml7396_rx_len(ng_ml7396_t *dev);

/**
 * @brief   Read a chunk of data from the receive buffer of the given device
 *
 * @param[in]  dev          device to read from
 * @param[out] data         buffer to write data to
 * @param[in]  len          number of bytes to read from device
 * @param[in]  offset       offset in the receive buffer
 */
void ng_ml7396_rx_read(ng_ml7396_t *dev, uint8_t *data, size_t len,
                          size_t offset);

#ifdef __cplusplus
}
#endif

#endif /* NG_ML7396_H_ */
/** @} */
