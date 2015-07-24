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
 * @brief       Netdev adaption for the ML7396 driver
 *
 *
 * @}
 */

#include <stdio.h>
#include <unistd.h>

#include "net/eui64.h"
#include "net/ng_ieee802154.h"
#include "net/ng_netbase.h"
#include "ng_ml7396.h"
#include "ng_ml7396_netdev.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_registers.h"

#include "ieee802154_frame.h"

#include "crc.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

extern void dump_buffer(char *buf, size_t len);

static size_t _make_ack_frame_hdr(ng_ml7396_t *dev, uint8_t *buf,
                                  uint64_t dst_addr, uint8_t seq_nr)
{
    int pos = 0;

    /* we are building a data frame here */
    buf[0] = NG_IEEE802154_FCF_TYPE_ACK;
    buf[1] = NG_IEEE802154_FCF_VERS_V1;

    /* fill in destination PAN ID */
    pos = 3;
    buf[pos++] = (uint8_t)((dev->pan) & 0xff);
    buf[pos++] = (uint8_t)((dev->pan) >> 8);

    buf[1] |= NG_IEEE802154_FCF_DST_ADDR_LONG;
    uint8_t *_dst_addr = (uint8_t *) &dst_addr;

    for (int i = 7;  i >= 0; i--) {
        buf[pos++] = _dst_addr[i];
    }

    /* set sequence number */
    buf[2] = seq_nr;
    /* return actual header length */
    return pos;
}

/* TODO: generalize and move to ng_ieee802154 */
/* TODO: include security header implications */
static size_t _get_frame_hdr_len(uint8_t *mhr)
{
    uint8_t tmp;
    size_t len = 3;

    /* figure out address sizes */
    tmp = (mhr[1] & NG_IEEE802154_FCF_DST_ADDR_MASK);
    if (tmp == NG_IEEE802154_FCF_DST_ADDR_SHORT) {
        len += 4;
    }
    else if (tmp == NG_IEEE802154_FCF_DST_ADDR_LONG) {
        len += 10;
    }
    else if (tmp != NG_IEEE802154_FCF_DST_ADDR_VOID) {
        return 0;
    }
    tmp = (mhr[1] & NG_IEEE802154_FCF_SRC_ADDR_MASK);
    if (tmp == NG_IEEE802154_FCF_SRC_ADDR_VOID) {
        return len;
    }
    else {
        if (!(mhr[0] & NG_IEEE802154_FCF_PAN_COMP)) {
            len += 2;
        }
        if (tmp == NG_IEEE802154_FCF_SRC_ADDR_SHORT) {
            return (len + 2);
        }
        else if (tmp == NG_IEEE802154_FCF_SRC_ADDR_LONG) {
            return (len + 8);
        }
    }
    return 0;
}

/* TODO: generalize and move to ng_ieee802154 */
static ng_pktsnip_t *_make_netif_hdr(uint8_t *mhr)
{
    uint8_t tmp;
    uint8_t *addr;
    uint8_t src_len, dst_len;
    ng_pktsnip_t *snip;
    ng_netif_hdr_t *hdr;

    /* figure out address sizes */
    tmp = mhr[1] & NG_IEEE802154_FCF_SRC_ADDR_MASK;
    if (tmp == NG_IEEE802154_FCF_SRC_ADDR_SHORT) {
        src_len = 2;
    }
    else if (tmp == NG_IEEE802154_FCF_SRC_ADDR_LONG) {
        src_len = 8;
    }
    else if (tmp == 0) {
        src_len = 0;
    }
    else {
        return NULL;
    }
    tmp = mhr[1] & NG_IEEE802154_FCF_DST_ADDR_MASK;
    if (tmp == NG_IEEE802154_FCF_DST_ADDR_SHORT) {
        dst_len = 2;
    }
    else if (tmp == NG_IEEE802154_FCF_DST_ADDR_LONG) {
        dst_len = 8;
    }
    else if (tmp == 0) {
        dst_len = 0;
    }
    else {
        return NULL;
    }
    /* allocate space for header */
    snip = ng_pktbuf_add(NULL, NULL, sizeof(ng_netif_hdr_t) + src_len + dst_len,
                         NG_NETTYPE_NETIF);
    if (snip == NULL) {
        return NULL;
    }
    /* fill header */
    hdr = (ng_netif_hdr_t *)snip->data;
    ng_netif_hdr_init(hdr, src_len, dst_len);
    if (dst_len > 0) {
        tmp = 5 + dst_len;
        addr = ng_netif_hdr_get_dst_addr(hdr);
        for (int i = 0; i < dst_len; i++) {
            addr[i] = mhr[5 + (dst_len - i) - 1];
        }
    }
    else {
        tmp = 3;
    }
    if (!(mhr[0] & NG_IEEE802154_FCF_PAN_COMP)) {
        tmp += 2;
    }
    if (src_len > 0) {
        addr = ng_netif_hdr_get_src_addr(hdr);
        for (int i = 0; i < src_len; i++) {
            addr[i] = mhr[tmp + (src_len - i) - 1];
        }
    }
    return snip;
}

static void _ng_ml7396_ack_received(ng_netdev_t *netdev, ieee802154_frame_t *frame)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) netdev;
    uint64_t addr;
    msg_t msg;

    if (frame->fcf.dest_addr_m != IEEE_802154_LONG_ADDR_M)
        return;

    addr = ng_ml7396_get_addr_long(dev);

    if (memcmp(&addr, frame->dest_addr, sizeof(uint64_t)) == 0) {
        msg.type = MSG_ACK_RECEIVED;
        msg.content.value = frame->seq_nr;
        msg_send(&msg, dev->tx_pid);

        printf("%s: ACK received.\n", __FUNCTION__);
    }

    return;
}

static void _ng_ml7396_send_ack(ng_netdev_t *netdev, ieee802154_frame_t *frame)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) netdev;
    msg_t msg;
    ng_pktsnip_t *pkt;
    uint16_t panid;
    uint64_t addr;

    if (frame->fcf.ack_req == 0) {
        return;
    }

    panid = ng_ml7396_get_pan(dev);
    addr = ng_ml7396_get_addr_long(dev);

    if ((panid == frame->dest_pan_id) &&
        (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
        memcmp(frame->dest_addr, &addr, 8) == 0) {

        ng_ml7396_send_ack(dev, frame);
        printf("** SEND_ACK\n");
    }

    return;
}

static int _send(ng_netdev_t *netdev, ng_pktsnip_t *pkt)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) netdev;
    msg_t msg, reply;

    if (pkt == NULL) {
        return -ENOMSG;
    }

    if (dev == NULL) {
        ng_pktbuf_release(pkt);
        return -ENODEV;
    }

    msg.type = NG_NETAPI_MSG_TYPE_SND;
    msg.content.ptr = (char *) pkt;

    msg_send_receive(&msg, &reply, dev->tx_pid);

    return (int) msg.content.value;
}

static int _receive_data(ng_ml7396_t *dev, uint32_t status)
{
    ieee802154_frame_t frame;
    uint8_t mhr[NG_IEEE802154_MAX_HDR_LEN], crc_buf[2];
    size_t pkt_len, hdr_len;
    ng_pktsnip_t *hdr, *payload = NULL;
    ng_netif_hdr_t *netif;
    int int_pend[2] = {0, 0};
    int page, complete, crc_ok, clear_fifo;
    uint16_t phy_header, crc, crc_received;
    static int last_page = -1;

    clear_fifo = 0;

    /* detect FIFO page */
    if (status & (INT_RXFIFO0_DONE | INT_RXFIFO0_CRCERR)) {
        int_pend[0] = 1;
    }
    if (status & (INT_RXFIFO1_DONE | INT_RXFIFO1_CRCERR)) {
        int_pend[1] = 1;
    }

    if ((int_pend[0] == 1) && (int_pend[1] == 1)) {
        DEBUG("%s: Pending page 0 and 1.\n", __FUNCTION__);
        if ((last_page == -1) || (last_page == 1)) {
            page = 0;
        }
        else {
            page = 1;
        }
    }
    else if (int_pend[0] == 1) {
        page = 0;
    }
    else if (int_pend[1] == 1) {
        page = 1;
    }
    else {
        DEBUG("%s: page ???\n", __FUNCTION__);
        return -1;
    }

    if (status & (INT_RXFIFO0_DONE | INT_RXFIFO1_DONE)) {
        complete = 1;
    }
    else {
        complete = 0;
    }

    if (status & (INT_RXFIFO0_CRCERR | INT_RXFIFO1_CRCERR)) {
        crc_ok = 0;
    }
    else {
        crc_ok = 1;
    }

    if (complete == 0 && crc_ok == 0) {
        printf("%s: [%d] crcerr, return\n",
               __FUNCTION__, page);

        clear_fifo = 1;
        goto ret;
    }

    /* --- Begin ML7396 FIFO critical section --- */
    ng_ml7396_lock(dev);

    /* read PHR (2-octets) */
    ng_ml7396_rx_read(dev, (uint8_t *) &phy_header, 2);

    pkt_len = (int) (NTOHS(phy_header) & 0xff) - 2;

    if (pkt_len < 10) {
        printf("%s: ??? pkt_len = %d, intstat = 0x%08x, page = %d\n",
               __FUNCTION__, pkt_len, (unsigned int) status, page);
        clear_fifo = 1;

        ng_ml7396_unlock(dev);

        goto ret;
    }

    /* get FCF field and compute 802.15.4 header length */
    ng_ml7396_rx_read(dev, mhr, 2);
    hdr_len = _get_frame_hdr_len(mhr);

    if (hdr_len == 0) {
        DEBUG("[ng_ml7396] error: unable parse incoming frame header\n");
        ng_ml7396_unlock(dev);

        goto ret;
    }

    /* read the rest of the header and parse the netif header from it */
    ng_ml7396_rx_read(dev, &(mhr[2]), hdr_len - 2);
    hdr = _make_netif_hdr(mhr);
    if (hdr == NULL) {
        DEBUG("[ng_ml7396] error: unable to allocate netif header\n");
        ng_ml7396_unlock(dev);

        goto ret;
    }

    /* fill missing fields in netif header */
    netif = (ng_netif_hdr_t *) hdr->data;
    netif->if_pid = dev->mac_pid;

    /* allocate payload */
    payload = ng_pktbuf_add(hdr, NULL, (pkt_len - hdr_len), dev->proto);
    if (payload == NULL) {
        DEBUG("[ng_ml7396] error: unable to allocate incoming payload\n");
        ng_pktbuf_release(hdr);
        ng_ml7396_unlock(dev);

        goto ret;
    }

    /* copy payload */
    printf("*** %s: payload data: %p, size: %d\n", __FUNCTION__,
           payload->data, payload->size);
    ng_ml7396_rx_read(dev, payload->data, payload->size);

    /* read CRC */
    ng_ml7396_rx_read(dev, crc_buf, 2);

    /* read ED level */
    ng_ml7396_rx_read(dev, &(netif->lqi), 1);

    /* check CRC */
    crc = crc_ccitt(0, mhr, hdr_len);
    crc = crc_ccitt(crc, payload->data, payload->size);
    crc_received = crc_buf[0] | crc_buf[1] << 8;

    if (crc == crc_received) {
        crc_ok = 1;

        //dump_buffer(mhr, hdr_len);
        //dump_buffer(payload->data, payload->size);
        //dump_buffer(crc_buf, 2);
    }
    else {
        printf("! [%d] CRC(received) %02x%02x, calc: %04x, "
               "status: 0x%08x, hdr_len: %d, payload->size: %d\n",
               page, crc_buf[1], crc_buf[0], crc,
               (unsigned int) status, hdr_len, payload->size);

        dump_buffer(mhr, hdr_len);
        dump_buffer(payload->data, payload->size);
        dump_buffer(crc_buf, 2);

        crc_ok = 0;
    }

    if (crc_ok == 0) {
        clear_fifo = 1;

        ng_pktbuf_release(hdr);
        ng_ml7396_unlock(dev);

        goto ret;
    }

    ieee802154_frame_read(mhr, &frame, hdr_len);

    printf("------ IEEE802.15.4 frame ------\n");
    printf("frame_type: %d\n", frame.fcf.frame_type);
    printf("ack_req:    %d\n", frame.fcf.ack_req);

    if (frame.fcf.frame_type == IEEE_802154_ACK_FRAME) {
        ng_ml7396_unlock(dev);

        /* release packet */
        ng_pktbuf_release(payload);
        _ng_ml7396_ack_received(dev, &frame);
    } else {
        _ng_ml7396_send_ack(dev, &frame);

        ng_ml7396_unlock(dev);

        /* finish up and send data to upper layers */
        if (dev->event_cb) {
            dev->event_cb(NETDEV_EVENT_RX_COMPLETE, payload);
        }
    }

ret:
    /* clear interrupts */
    _ng_ml7396_clear_rx_interrupts(dev, page, clear_fifo);

    last_page = page;
    return page;
}

static int _set_state(ng_ml7396_t *dev, ng_netconf_state_t state)
{
    uint8_t val = 0xff;

    switch (state) {
        case NETCONF_STATE_SLEEP:
            val = SET_TRX_OFF;
            break;

        case NETCONF_STATE_IDLE:
        case NETCONF_STATE_RX:
            val = SET_RX_ON;
            break;

        case NETCONF_STATE_TX:
            if (dev->options & NG_ML7396_OPT_PRELOADING) {
                ng_ml7396_tx_exec(dev);
            }
            break;

        case NETCONF_STATE_RESET:
            ng_ml7396_reset(dev);
            break;

        default:
            return -ENOTSUP;
    }

    if (val != 0xff) {
        ng_ml7396_reg_write(dev, ML7396_REG_RF_STATUS, val);
    }

    return sizeof(ng_netconf_state_t);
}

ng_netconf_state_t _get_state(ng_ml7396_t *dev)
{
    uint8_t state;

    state = ng_ml7396_reg_read(dev, ML7396_REG_RF_STATUS) & 0xf0;

    switch (state) {
        case STAT_RX_ON:
            return NETCONF_STATE_RX;

        case STAT_TX_ON:
            return NETCONF_STATE_TX;

        case STAT_TRX_OFF:
        default:
            return NETCONF_STATE_IDLE;
    }
}

static int _get(ng_netdev_t *device, ng_netconf_opt_t opt,
                void *val, size_t max_len)
{
    if (device == NULL) {
        return -ENODEV;
    }
    ng_ml7396_t *dev = (ng_ml7396_t *) device;

    switch (opt) {
        case NETCONF_OPT_ADDRESS:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *) val) = ng_ml7396_get_addr_short(dev);
            return sizeof(uint16_t);

        case NETCONF_OPT_ADDRESS_LONG:
            if (max_len < sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            *((uint64_t *) val) = ng_ml7396_get_addr_long(dev);
            DEBUG("%s: NETCONF_OPT_ADDRESS_LONG -> %d\n",
                  __FUNCTION__, sizeof(uint64_t));
            return sizeof(uint64_t);

        case NETCONF_OPT_ADDR_LEN:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *) val) = 8;
            return sizeof(uint16_t);

        case NETCONF_OPT_SRC_LEN:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (dev->options & NG_ML7396_OPT_SRC_ADDR_LONG) {
                DEBUG("%s: ADDR_LONG\n", __FUNCTION__);
                *((uint16_t *) val) = 8;
            }
            else {
                DEBUG("%s: !ADDR_LONG\n", __FUNCTION__);
                *((uint16_t *) val) = 2;
            }
            return sizeof(uint16_t);

        case NETCONF_OPT_NID:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *) val) = dev->pan;
            return sizeof(uint16_t);

        case NETCONF_OPT_IPV6_IID:
              if (max_len < sizeof(eui64_t)) {
                  return -EOVERFLOW;
              }
              if (dev->options & NG_ML7396_OPT_SRC_ADDR_LONG) {
                  uint64_t addr = ng_ml7396_get_addr_long(dev);
                  ng_ieee802154_get_iid(val, (uint8_t *)&addr, 8);
              }
              else {
                  uint16_t addr = ng_ml7396_get_addr_short(dev);
                  ng_ieee802154_get_iid(val, (uint8_t *)&addr, 2);
              }
              return sizeof(eui64_t);

        case NETCONF_OPT_PROTO:
            if (max_len < sizeof(ng_nettype_t)) {
                return -EOVERFLOW;
            }
            *((ng_nettype_t *) val) = dev->proto;
            return sizeof(ng_nettype_t);

        case NETCONF_OPT_CHANNEL:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            ((uint8_t *) val)[1] = 0;
            ((uint8_t *) val)[0] = ng_ml7396_get_chan(dev);
            return sizeof(uint16_t);

        case NETCONF_OPT_MAX_PACKET_SIZE:
            if (max_len < sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *) val) = NG_ML7396_MAX_PKT_LENGTH;
            return sizeof(uint16_t);

        case NETCONF_OPT_STATE:
            if (max_len < sizeof(ng_netconf_state_t)) {
                return -EOVERFLOW;
            }
            *((ng_netconf_state_t *) val) = _get_state(dev);
            break;

        case NETCONF_OPT_PRELOADING:
            if (dev->options & NG_ML7396_OPT_PRELOADING) {
                *((ng_netconf_enable_t *) val) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *) val) = NETCONF_DISABLE;
            }
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_AUTOACK:
            if (dev->options & NG_ML7396_OPT_AUTOACK) {
                *((ng_netconf_enable_t *) val) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *) val) = NETCONF_DISABLE;
            }
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RETRANS:
            if (max_len < sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *) val) = ng_ml7396_get_max_retries(dev);
            return sizeof(uint8_t);

        case NETCONF_OPT_PROMISCUOUSMODE:
            if (dev->options & NG_ML7396_OPT_PROMISCUOUS) {
                *((ng_netconf_enable_t *) val) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *) val) = NETCONF_DISABLE;
            }
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RAWMODE:
            if (dev->options & NG_ML7396_OPT_RAWDUMP) {
                *((ng_netconf_enable_t *) val) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *) val) = NETCONF_DISABLE;
            }
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_IS_CHANNEL_CLR:
            if (ng_ml7396_cca(dev)) {
                *((ng_netconf_enable_t *) val) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *) val) = NETCONF_DISABLE;
            }
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RX_START_IRQ:
            *((ng_netconf_enable_t *) val) =
                !!(dev->options & NG_ML7396_OPT_TELL_RX_START);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RX_END_IRQ:
            *((ng_netconf_enable_t *) val) =
                !!(dev->options & NG_ML7396_OPT_TELL_RX_END);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_TX_START_IRQ:
            *((ng_netconf_enable_t *) val) =
                !!(dev->options & NG_ML7396_OPT_TELL_TX_START);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_TX_END_IRQ:
            *((ng_netconf_enable_t *) val) =
                !!(dev->options & NG_ML7396_OPT_TELL_TX_END);
            return sizeof(ng_netconf_enable_t);

        default:
            return -ENOTSUP;
    }

    return 0;
}

static int _set(ng_netdev_t *device, ng_netconf_opt_t opt,
                void *val, size_t len)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) device;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETCONF_OPT_ADDRESS:
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            ng_ml7396_set_addr_short(dev, *((uint16_t *) val));
            return sizeof(uint16_t);

        case NETCONF_OPT_ADDRESS_LONG:
            if (len > sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            ng_ml7396_set_addr_long(dev, *((uint64_t *) val));
            return sizeof(uint64_t);

        case NETCONF_OPT_SRC_LEN:
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (*((uint16_t *) val) == 2) {
                ng_ml7396_set_option(dev, NG_ML7396_OPT_SRC_ADDR_LONG,
                                     false);
            }
            else if (*((uint16_t *) val) == 8) {
                ng_ml7396_set_option(dev, NG_ML7396_OPT_SRC_ADDR_LONG,
                                     true);
            }
            else {
                return -ENOTSUP;
            }
            return sizeof(uint16_t);

        case NETCONF_OPT_NID:
            if (len > sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            ng_ml7396_set_pan(dev, *((uint16_t *) val));
            return sizeof(uint16_t);

        case NETCONF_OPT_CHANNEL:
            if (len != sizeof(uint16_t)) {
                return -EINVAL;
            }
            uint8_t chan = ((uint8_t *) val)[0];
            if ((chan < NG_ML7396_MIN_CHANNEL) ||
                (chan > NG_ML7396_MAX_CHANNEL) ||
                ((chan % 2) == 0)) {
                return -ENOTSUP;
            }
            ng_ml7396_set_chan(dev, chan);
            return sizeof(uint16_t);

        case NETCONF_OPT_STATE:
            if (len > sizeof(ng_netconf_state_t)) {
                return -EOVERFLOW;
            }
            return _set_state(dev, *((ng_netconf_state_t *) val));

        case NETCONF_OPT_AUTOACK:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_AUTOACK,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RETRANS:
            if (len > sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            ng_ml7396_set_max_retries(dev, *((uint8_t *) val));
            return sizeof(uint8_t);

        case NETCONF_OPT_PRELOADING:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_PRELOADING,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_PROMISCUOUSMODE:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_PROMISCUOUS,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RAWMODE:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_RAWDUMP,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RX_START_IRQ:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_TELL_RX_START,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_RX_END_IRQ:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_TELL_RX_END,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_TX_START_IRQ:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_TELL_TX_START,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        case NETCONF_OPT_TX_END_IRQ:
            ng_ml7396_set_option(dev, NG_ML7396_OPT_TELL_TX_END,
                                 ((bool *) val)[0]);
            return sizeof(ng_netconf_enable_t);

        default:
            return -ENOTSUP;
    }

    return 0;
}

static int _add_event_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    static int reset = 0;

    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb) {
        return -ENOBUFS;
    }

    dev->event_cb = cb;

    if (reset == 0) {
        reset = 1;

        /* reset device to default values and put it into RX state */
        ng_ml7396_reset(dev);
    }

    return 0;
}

static int _rem_event_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb != cb) {
        return -ENOENT;
    }

    dev->event_cb = NULL;
    return 0;
}

static void _isr_event(ng_netdev_t *netdev, uint32_t event_type)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) netdev;
    uint32_t status, enable;
    int page;

    enable = ng_ml7396_get_interrupt_enable(dev);
    status = ng_ml7396_get_interrupt_status(dev);

    /* FIFO 0/1 RX Done/CRCError interrupt */
    if (status & (INT_RXFIFO0_DONE | INT_RXFIFO1_DONE |
                  INT_RXFIFO0_CRCERR | INT_RXFIFO1_CRCERR |
                  INT_RXFIFO_ERR)) {
        page = _receive_data(dev, status);

        if (page == 0) {
            if (status & INT_RXFIFO0_DONE) {
                status &= ~INT_RXFIFO0_DONE;
            }
            if (status & INT_RXFIFO0_CRCERR) {
                status &= ~INT_RXFIFO0_CRCERR;
            }
        }
        else if (page == 1) {
            if (status & INT_RXFIFO1_DONE) {
                status &= ~INT_RXFIFO1_DONE;
            }
            if (status & INT_RXFIFO1_CRCERR) {
                status &= ~INT_RXFIFO1_CRCERR;
            }
        }

        if (status & INT_RXFIFO_ERR) {
            status &= ~INT_RXFIFO_ERR;
        }
    }

    if (status & enable) {
        /* wakeup thread */
        //ng_ml7396_wakeup_interrupt(status);

        enable &= ~status;
    }

    enable |= (INT_RXFIFO1_CRCERR | INT_RXFIFO0_CRCERR |
               INT_RXFIFO1_DONE | INT_RXFIFO0_DONE |
               INT_RXFIFO_ERR);

    /* re-enable interrupt */
    ng_ml7396_set_interrupt_mask(dev, ML7396_INT_ALL);
    ng_ml7396_set_interrupt_enable(dev, enable);
}

const ng_netdev_driver_t ng_ml7396_driver = {
    .send_data = _send,
    .add_event_callback = _add_event_cb,
    .rem_event_callback = _rem_event_cb,
    .get = _get,
    .set = _set,
    .isr_event = _isr_event,
};
