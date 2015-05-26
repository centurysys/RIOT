/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ml7396 ml7396
 * @ingroup     drivers
 * @brief       Device driver for the Lapis ML7396 radio
 * @{
 *
 * @file
 * @brief       Interface definition for the ML7396 device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef ML7396_H_
#define ML7396_H_

#include <stdio.h>
#include <stdint.h>

#include "mutex.h"
#include "kernel_types.h"
#include "board.h"
#include "radio/types.h"
#include "ieee802154_frame.h"
#include "ml7396_settings.h"
#include "periph/gpio.h"
#include "netdev/802154.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum length of a frame on ml7396
 */
#define ML7396_MAX_PKT_LENGTH    (255)

/**
 * @brief Maximum payload length
 * @details Assuming intra PAN long address mode
 *          results in 2 bytes FCF
 *          + 1 bytes SEQNr
 *          + 2 bytes PAN Id
 *          + 8 bytes destination address
 *          + 8 bytes source address
 */
#define ML7396_MAX_DATA_LENGTH   (234)

/**
 * @brief Broadcast address
 */
#define ML7396_BROADCAST_ADDRESS (0xFFFF)

/**
 * @brief ml7396's lowest supported channel
 */
#define ML7396_MIN_CHANNEL       (33)

/**
 * @brief ml7396's highest supported channel
 */
#define ML7396_MAX_CHANNEL       (59)

/**
 *  @brief Structure to represent a ml7396 packet.
 */
typedef struct __attribute__((packed))
{
    /** @{ */
    uint8_t length;             /**< the length of the frame of the frame including fcs*/
    ieee802154_frame_t frame;   /**< the ieee802154 frame */
    uint8_t rssi;               /**< the rssi value */
    uint8_t crc;                /**< 1 if crc was successfull, 0 otherwise */
    uint8_t lqi;                /**< the link quality indicator */
    uint8_t ed;                 /**< the ED (energy detection) value */
    /** @} */
} ml7396_packet_t;

extern netdev_t ml7396_netdev;   /**< netdev representation of this driver */

/**
 * @brief States to be assigned to `driver_state`
 * @{
 */
#define AT_DRIVER_STATE_DEFAULT     (0)
#define AT_DRIVER_STATE_SENDING     (1)
/** @} */

/**
 * @brief To keep state inside of ml7396 driver
 * @details This variable is used to determine if a TRX_END IRQ from
 *          the radio transceiver has to be interpreted as end of
 *          sending or reception.
 */
extern uint8_t driver_state;

/**
 * @brief Initialize the ml7396 transceiver
 */
int ml7396_initialize(netdev_t *dev);

void ml7396_reset(void);
void ml7396_phy_reset(void);

#ifdef MODULE_TRANSCEIVER
/**
 * @brief Init the ml7396 for use with RIOT's transceiver module.
 *
 * @param[in] tpid The PID of the transceiver thread.
 */

void ml7396_init(kernel_pid_t tpid);
#endif

/**
 * @brief Turn ml7396 on.
 *
 * @return 1 if the radio was correctly turned on; 0 otherwise.
 */
int ml7396_on(void);

/**
 * @brief Turn ml7396 off.
 */
void ml7396_off(void);

/**
 * @brief Indicate if the ml7396 is on.
 *
 * @return 1 if the radio transceiver is on (active); 0 otherwise.
 */
int ml7396_is_on(void);

/**
 * @brief Switches the ml7396 into receive mode.
 */
void ml7396_switch_to_rx(void);

void ml7396_switch_to_tx(void);

int ml7396_switch_to_trx_off(void);

/**
 * @brief Turns monitor (promiscuous) mode on or off.
 *
 * @param[in] mode The desired mode:
 *                 1 for monitor (promiscuous) mode;
 *                 0 for normal (auto address-decoding) mode.
 */
void ml7396_set_monitor(int mode);

/**
 * @brief Indicate if the ml7396 is in monitor (promiscuous) mode.
 *
 * @return 1 if the transceiver is in monitor (promiscuous) mode;
 *         0 if it is in normal (auto address-decoding) mode.
 */
int ml7396_get_monitor(void);

/**
 * @brief Set the channel of the ml7396.
 *
 * @param[in] chan The desired channel, valid channels are from 11 to 26.
 *
 * @return The tuned channel after calling, or -1 on error.
 */
int ml7396_set_channel(unsigned int chan);

/**
 * @brief Get the channel of the ml7396.
 *
 * @return The tuned channel.
 */
unsigned int ml7396_get_channel(void);

/**
 * @brief Sets the short address of the ml7396.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint16_t ml7396_set_address(uint16_t addr);

/**
 * @brief Gets the current short address of the ml7396.
 *
 * @return The current short address.
 */
uint16_t ml7396_get_address(void);

/**
 * @brief Sets the IEEE long address of the ml7396.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint64_t ml7396_set_address_long(uint64_t addr);

/**
 * @brief Gets the current IEEE long address of the ml7396.
 *
 * @return The current IEEE long address.
 */
uint64_t ml7396_get_address_long(void);

void ml7396_get_address_long_buf(uint8_t *buf);

/**
 * @brief Sets the pan ID of the ml7396.
 *
 * @param[in] pan The desired pan ID.
 *
 * @return The set pan ID after calling.
 */
uint16_t ml7396_set_pan(uint16_t pan);

/**
 * @brief Gets the current IEEE long address of the ml7396.
 *
 * @return The current IEEE long address.
 */
uint16_t ml7396_get_pan(void);

/**
 * @brief Sets the output (TX) power of the ml7396.
 *
 * @param[in] pow The desired TX (output) power in dBm,
 *                 valid values are -25 to 0; other values
 *                 will be "saturated" into this range.
 *
 * @return The set TX (output) power after calling.
 */
int ml7396_set_tx_power(int pow);

/**
 * @brief Gets the current output (TX) power of the ml7396.
 *
 * @return The current TX (output) power.
 */
int ml7396_get_tx_power(void);

/**
 * @brief Checks if the radio medium is available/clear to send
 *         ("Clear Channel Assessment" a.k.a. CCA).
 *
 * @return a 1 value if radio medium is clear (available),
 *         a 0 value otherwise.
 *
 */
int ml7396_channel_is_clear(netdev_t *dev);

/**
 * @brief Sets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function for 802.15.4 packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int ml7396_add_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Unsets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int ml7396_rem_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Sets a function called back when a data packet is received.
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function for 802.15.4 data packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int ml7396_add_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb);

/**
 * @brief Unsets a function called back when a data packet is received.
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int ml7396_rem_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb);

/**
 * @brief RX handler, process data from the RX FIFO.
 *
 */
int ml7396_rx_handler(uint32_t status);

/**
 * @brief Prepare the ml7396 TX buffer to send with the given packet.
 *
 * @param[in] dev  The network device to operate on. (Currently not used)
 * @param[in] kind Kind of packet to transmit.
 * @param[in] dest Address of the node to which the packet is sent.
 * @param[in] use_long_addr 1 to use the 64-bit address mode
 *                          with *dest* param; 0 to use
 *                          "short" PAN-centric mode.
 * @param[in] wants_ack 1 to request an acknowledgement
 *                      from the receiving node for this packet;
 *                      0 otherwise.
 * @param[in] upper_layer_hdrs  header data from higher network layers from
 *                              highest to lowest layer. Must be prepended to
 *                              the data stream by the network device. May be
 *                              NULL if there are none.
 * @param[in] buf Pointer to the buffer containing the payload
 *                of the 802.15.4 packet to transmit.
 *                The frame header (i.e.: FCS, sequence number,
 *                src and dest PAN and addresses) is inserted
 *                using values in accord with *kind* parameter
 *                and transceiver configuration.
 * @param[in] len Length (in bytes) of the outgoing packet payload.
 *
 * @return @ref netdev_802154_tx_status_t
 */
netdev_802154_tx_status_t ml7396_load_tx_buf(netdev_t *dev,
                                             netdev_802154_pkt_kind_t kind,
                                             netdev_802154_node_addr_t *dest,
                                             int use_long_addr,
                                             int wants_ack,
                                             netdev_hlist_t *upper_layer_hdrs,
                                             void *buf,
                                             unsigned int len);

/**
 * @brief Transmit the data loaded into the ml7396 TX buffer.
 *
 * @param[in] dev The network device to operate on. (Currently not used)
 *
 * @return @ref netdev_802154_tx_status_t
 */
netdev_802154_tx_status_t ml7396_transmit_tx_buf(netdev_t *dev);

/**
 * @brief Send function, sends a ml7396_packet_t over the air.
 *
 * @param[in] *packet The Packet which will be send.
 *
 * @return The count of bytes which are send or -1 on error
 *
 */
int16_t ml7396_send(ml7396_packet_t *packet);
int16_t ml7396_send_raw(char *buf, int len);
//int16_t ml7396_send_raw2(char *buf, int len);

int16_t ml7396_send_ack(ieee802154_frame_t *frame, int enhanced);


/**
 * RX Packet Buffer, read from the transceiver, filled by the ml7396_rx_handler.
 */
extern ml7396_packet_t ml7396_rx_buffer[ML7396_RX_BUF_SIZE];

/**
 * Get ml7396's status byte
 */
uint8_t ml7396_get_status(void);

/**
 * Get ml7396's TRAC status byte
 */
uint8_t ml7396_get_trac_status(void);

/**
 * ml7396 low-level radio driver definition.
 */
extern const netdev_802154_driver_t ml7396_driver;

int ml7396_wait_interrupt(uint32_t interrupts, int clear, mutex_t *mutex);

void ml7396_lock(void);
void ml7396_unlock(void);


#ifdef __cplusplus
}
#endif

#endif /* ML7396_H_ */
/** @} */
