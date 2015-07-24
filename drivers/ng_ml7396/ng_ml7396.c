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
 * @brief       Implementation of public functions for ML7396 driver
 *
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "hwtimer.h"
#include "periph/cpuid.h"
#include "periph/random.h"
#include "byteorder.h"
#include "net/ng_ieee802154.h"
#include "net/ng_netbase.h"

#include "ng_ml7396_registers.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_netdev.h"
#include "ng_ml7396_tx.h"
#include "ng_ml7396_irq.h"

#include "crc.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#if ENABLE_DEBUG
static ng_ml7396_t *dev_debug;
#endif

/* CSMA parameters */
static const uint32_t aUnitBackoffPeriod = 1130;
static const int macMaxCSMABackoffs = 4;
static const int macMinBE = 8;
static const int macMaxBE = 8;


struct ng_ml7396_setting {
    uint16_t reg;
    uint8_t val;
};

static struct ng_ml7396_setting rf_settings[] = {
    /*
     * Bank 0
     */
    { ML7396_REG_RST_SET, 0x00 },
    { ML7396_REG_RATE_SET1, 0x00 },
    { ML7396_REG_RATE_SET2, 0x00 },
    { ML7396_REG_ADC_CLK_SET, 0xd3 },
    { ML7396_REG_GAIN_MtoL, 0x1e },
    { ML7396_REG_GAIN_LtoH, 0x02 },
    { ML7396_REG_GAIN_HtoM, 0x9e },
    { ML7396_REG_GAIN_MtoH, 0x02 },
    { ML7396_REG_RSSI_ADJ_M, 0x16 },
    { ML7396_REG_RSSI_ADJ_L, 0x2d },
    { ML7396_REG_RSSI_STABLE_TIME, 0x12 },
    { ML7396_REG_RSSI_VAL_ADJ, 0xd4 },
    { ML7396_REG_IF_FREQ_AFC_H, 0x14 },
    { ML7396_REG_IF_FREQ_AFC_L, 0x7a },
    { ML7396_REG_BPF_AFC_ADJ_H, 0x02 },
    { ML7396_REG_BPF_AFC_ADJ_L, 0x6f },
    { ML7396_REG_AFC_CNTRL, 0x01 },
    { ML7396_REG_PREAMBLE_SET, 0xaa },
    { ML7396_REG_SFD1_SET1, 0x09 },
    { ML7396_REG_SFD1_SET2, 0x72 },
    { ML7396_REG_SFD1_SET3, 0xf6 },
    { ML7396_REG_SFD1_SET4, 0x72 },
    { ML7396_REG_SFD2_SET1, 0x5e },
    { ML7396_REG_SFD2_SET2, 0x70 },
    { ML7396_REG_SFD2_SET3, 0xc6 },
    { ML7396_REG_SFD2_SET4, 0xb4 },
    { ML7396_REG_TX_PR_LEN, 0x0a },
    { ML7396_REG_RX_PR_LEN, 0x12 },
    { ML7396_REG_SYNC_CONDITION, 0x00 },
    { ML7396_REG_DATA_SET, 0x11 },
    { ML7396_REG_FEC_CRC_SET, 0x0a }, /* Disable CRC */
    /* CH#0 Freq */
    { ML7396_REG_CH0_FL, 0x00 },
    { ML7396_REG_CH0_FM, 0x00 },
    { ML7396_REG_CH0_FH, 0x0a },
    { ML7396_REG_CH0_NA, 0x61 },
    { ML7396_REG_CH_SPACE_L, 0x82 },
    { ML7396_REG_CH_SPACE_H, 0x2d },
    /* Enable channel setting (#0 - #13) */
    { ML7396_REG_CH_EN_L, 0xff },
    { ML7396_REG_CH_EN_H, 0x3f },
    /* set channel (default: #0, ch33) */
    { ML7396_REG_CH_SET, 0x00 },

    { ML7396_REG_F_DEV_L, 0xb0 },
    { ML7396_REG_F_DEV_H, 0x05 },
    { ML7396_REG_ACK_TIMER_EN, 0x20 },
    { ML7396_REG_PLL_MONO_DIO_SEL, 0x00 },
    { ML7396_REG_2DIV_GAIN_CNTRL, 0x02 },
    { ML7396_REG_2DIV_SEARCH, 0x16 },
    { ML7396_REG_2DIV_CNTRL, 0x0e },
    { ML7396_REG_2DIV_RSLT, 0x00 },
    { ML7396_REG_RF_CNTRL_SET, 0x00 },
    /* CCA */
    { ML7396_REG_CCA_LEVEL, 0x51 },
    { ML7396_REG_IDLE_WAIT_L, 0x00 },
    { ML7396_REG_IDLE_WAIT_H, 0x00 },
    /* ALARM */
    { ML7396_REG_TX_ALARM_LH, 0x00 },
    { ML7396_REG_TX_ALARM_HL, 0x00 },
    { ML7396_REG_PACKET_MODE_SET, 0x8b },

    /*
     * Bank 1
     */
    { ML7396_REG_DEMAND_SET, 0x00 },
    { ML7396_REG_PA_ADJ1, 0x77 },
    { ML7396_REG_PA_ADJ2, 0x3f },
    { ML7396_REG_PA_ADJ3, 0x9f },
    { ML7396_REG_SW_OUT_RAMP_ADJ, 0x00 },
    { ML7396_REG_PLL_CP_ADJ, 0x44 },
    { ML7396_REG_IF_FREQ_H, 0x14 },
    { ML7396_REG_IF_FREQ_L, 0x7a },
    { ML7396_REG_IF_FREQ_CCA_H, 0x1c },
    { ML7396_REG_IF_FREQ_CCA_L, 0x71 },
    { ML7396_REG_BPF_ADJ_H, 0x02 },
    { ML7396_REG_BPF_ADJ_L, 0x6f },
    { ML7396_REG_BPF_CCA_ADJ_H, 0x01 },
    { ML7396_REG_BPF_CCA_ADJ_L, 0x2b },
    { ML7396_REG_RSSI_LPF_ADJ, 0x1f },
    { ML7396_REG_PA_REG_FINE_ADJ, 0x10 },
    { ML7396_REG_IQ_MAG_ADJ, 0x08 },
    { ML7396_REG_IQ_PHASE_ADJ, 0x20 },
    { ML7396_REG_VCO_CAL_MIN_FL, 0xaa },
    { ML7396_REG_VCO_CAL_MIN_FM, 0xaa },
    { ML7396_REG_VCO_CAL_MIN_FH, 0x05 },
    { ML7396_REG_VCO_CAL_MAX_N, 0x0f },
    { ML7396_REG_PA_REG_ADJ1, 0x07 },
    { ML7396_REG_PA_REG_ADJ2, 0x07 },
    { ML7396_REG_PA_REG_ADJ3, 0x05 },
    { ML7396_REG_PDD_LD, 0x84 },
    { ML7396_REG_PLL_CTRL, 0x9f },
    { ML7396_REG_RX_ON_ADJ2, 0x02 },
    { ML7396_REG_LNA_GAIN_ADJ_M, 0x0f },
    { ML7396_REG_LNA_GAIN_ADJ_L, 0x01 },
    { ML7396_REG_MIN_GAIN_ADJ_M, 0xff },
    { ML7396_REG_MIN_GAIN_ADJ_L, 0xff },
    { ML7396_REG_BPF_GAIN_ADJ, 0x70 },
    { ML7396_REG_TX_OFF_ADJ1, 0x00 },
    { ML7396_REG_RSSI_SLOPE_ADJ, 0x01 },

    /*
     *  Bank 2
     */
    { ML7396_REG_NOISE_DET, 0x27 },
    { ML7396_REG_SYNC_MODE, 0x00 },
    { ML7396_REG_PA_ON_ADJ, 0x04 },
    { ML7396_REG_RX_ON_ADJ, 0x0a },
    { ML7396_REG_RXD_ADJ, 0x00 },
    { ML7396_REG_RATE_ADJ1, 0x01 },
    { ML7396_REG_RATE_ADJ2, 0x0f },
    { ML7396_REG_RAMP_CNTRL, 0x00 },
    { ML7396_REG_BPF_CAP1, 0x2c },
    { ML7396_REG_BPF_CAP2, 0xc0 },
    { ML7396_REG_BPF_ADJ1, 0x17 },
    { ML7396_REG_BPF_ADJ2, 0x17 },

    { 0xffff, 0xff } /* END */
};


static void _irq_handler(void *arg)
{
    msg_t msg;
    ng_ml7396_t *dev = (ng_ml7396_t *) arg;

    DEBUG("** IRQ, mac_pid = %d\n", dev->irq_pid);

    /* tell driver thread about the interrupt */
    msg.type = NG_NETDEV_MSG_TYPE_EVENT;
    msg_send(&msg, dev->irq_pid);
}

static void _ng_ml7396_spi_init(ng_ml7396_t *dev, spi_speed_t spi_speed)
{
    int res;

    res = spi_init_master(dev->spi, SPI_CONF_FIRST_RISING, spi_speed);

    if (res < 0) {
        printf("%s: spi_init_master failed.\n", __FUNCTION__);
        return;
    }

    gpio_init(dev->cs_pin, GPIO_DIR_OUT, GPIO_PULLUP);
    gpio_init(dev->reset_pin, GPIO_DIR_OUT, GPIO_PULLUP);

    if (dev->tcxo_pin) {
        gpio_init(dev->tcxo_pin, GPIO_DIR_OUT, GPIO_PULLUP);
        gpio_clear(dev->tcxo_pin);
    }
}

static void _ng_ml7396_gpio_interrupt_init(ng_ml7396_t *dev)
{
    gpio_init_int(dev->int_pin, GPIO_PULLUP, GPIO_FALLING,
                  (gpio_cb_t) _irq_handler, dev);

    ng_ml7396_set_interrupt_mask(dev, ML7396_INT_ALL);
}

static void _ng_ml7396_clk_init(ng_ml7396_t *dev)
{
    mutex_t mutex = MUTEX_INIT;

    ng_ml7396_set_interrupt_enable(dev, INT_CLK_STABLE);

    ng_ml7396_reg_write(dev, ML7396_REG_RST_SET, 0xff);
    ng_ml7396_reg_write(dev, ML7396_REG_CLK_SET, (CLK_Done | TCXO_EN |
                                                  CLK3_EN | CLK2_EN | CLK1_EN | CLK0_EN));

    mutex_lock(&mutex);
    _ng_ml7396_wait_interrupt(dev, INT_CLK_STABLE, /* clear = */ 1, &mutex);
}

static void _ng_ml7396_setup_rf_basic_settings(ng_ml7396_t *dev)
{
    struct ng_ml7396_setting *conf;

    for (conf = rf_settings; conf->reg != 0xffff; conf++) {
        ng_ml7396_reg_write(dev, conf->reg, conf->val);
    }
}

static void _ng_ml7396_rf_init(ng_ml7396_t *dev)
{
    uint8_t val;
    int16_t bpf_offset, bpf_cca_offset, sign;
    uint16_t bpf_val, bpf_cca_val;
    mutex_t mutex = MUTEX_INIT;

    _ng_ml7396_setup_rf_basic_settings(dev);

    /* get BPF_ADJ_OFFSET */
    val = ng_ml7396_reg_read(dev, ML7396_REG_BPF_ADJ_OFFSET);

    bpf_offset = (int16_t) (val & 0x7f);
    bpf_cca_offset = (int16_t) (((int16_t) (val & 0x7f)) * 48 / 100);

    sign = (val & 0x80) ? 1 : -1;

    bpf_val = 0x024a + (bpf_offset * sign);
    bpf_cca_val = 0x0119 + (bpf_cca_offset * sign);

    /* BPF adjust */
    ng_ml7396_reg_write(dev, ML7396_REG_BPF_ADJ_H, (bpf_val & 0xff00) >> 8);
    ng_ml7396_reg_write(dev, ML7396_REG_BPF_ADJ_L, (bpf_val & 0x00ff));

    ng_ml7396_reg_write(dev, ML7396_REG_BPF_AFC_ADJ_H, (bpf_val & 0xff00) >> 8);
    ng_ml7396_reg_write(dev, ML7396_REG_BPF_AFC_ADJ_L, (bpf_val & 0x00ff));

    ng_ml7396_reg_write(dev, ML7396_REG_BPF_CCA_ADJ_H, (bpf_cca_val & 0xff00) >> 8);
    ng_ml7396_reg_write(dev, ML7396_REG_BPF_CCA_ADJ_L, (bpf_cca_val & 0x00ff));

    ng_ml7396_set_interrupt_enable(dev, INT_VCO_DONE);

    /* start VCO calibration */
    ng_ml7396_reg_write(dev, ML7396_REG_VCO_CAL_START, 0x01);

    /* wait for calibration done. */
    mutex_lock(&mutex);
    _ng_ml7396_wait_interrupt(dev, INT_VCO_DONE, /* clear = */ 1, &mutex);

    ng_ml7396_set_interrupt_mask(dev, INT_VCO_DONE);
}

static void _ng_ml7396_backoff(ng_ml7396_t *dev)
{
    uint8_t rand;
    uint32_t backoff;

    random_read((char *) &rand, 1);
    backoff = aUnitBackoffPeriod * rand; /* usecs */

    if (backoff > 10) {
        usleep(backoff);
    }
}

static size_t _make_data_frame_hdr(ng_ml7396_t *dev, uint8_t *buf,
                                   ng_netif_hdr_t *hdr)
{
    int pos = 0;

    /* we are building a data frame here */
    buf[0] = NG_IEEE802154_FCF_TYPE_DATA;
    buf[1] = 0x88;      /* use short src and dst addresses as starting point */

    /* if unicast packet, then we also expect ACKs for this packet */
    if (!(hdr->flags & NG_NETIF_HDR_FLAGS_BROADCAST) &&
        !(hdr->flags & NG_NETIF_HDR_FLAGS_MULTICAST)) {
        buf[0] |= NG_IEEE802154_FCF_ACK_REQ;
    }

    /* fill in destination PAN ID */
    pos = 3;
    buf[pos++] = (uint8_t)((dev->pan) & 0xff);
    buf[pos++] = (uint8_t)((dev->pan) >> 8);

    /* fill in destination address */
    if (hdr->flags &
        (NG_NETIF_HDR_FLAGS_BROADCAST | NG_NETIF_HDR_FLAGS_MULTICAST)) {
        buf[pos++] = 0xff;
        buf[pos++] = 0xff;
    }
    else if (hdr->dst_l2addr_len == 2) {
        uint8_t *dst_addr = ng_netif_hdr_get_dst_addr(hdr);
        buf[pos++] = dst_addr[1];
        buf[pos++] = dst_addr[0];
    }
    else if (hdr->dst_l2addr_len == 8) {
        buf[1] |= 0x04;
        uint8_t *dst_addr = ng_netif_hdr_get_dst_addr(hdr);
        for (int i = 7;  i >= 0; i--) {
            buf[pos++] = dst_addr[i];
        }
    }
    else {
        /* unsupported address length */
        return 0;
    }

    /* fill in source PAN ID (if applicable */
    if (dev->options & NG_ML7396_OPT_USE_SRC_PAN) {
        buf[pos++] = (uint8_t)((dev->pan) & 0xff);
        buf[pos++] = (uint8_t)((dev->pan) >> 8);
    } else {
        buf[0] |= NG_IEEE802154_FCF_PAN_COMP;
    }

    /* fill in source address */
    if (dev->options & NG_ML7396_OPT_SRC_ADDR_LONG) {
        buf[1] |= 0x40;
        memcpy(&(buf[pos]), dev->addr_long, 8);
        pos += 8;
    }
    else {
        buf[pos++] = dev->addr_short[0];
        buf[pos++] = dev->addr_short[1];
    }

    /* set sequence number */
    buf[2] = dev->seq_nr++;
    /* return actual header length */
    return pos;
}


int ng_ml7396_init(ng_ml7396_t *dev, spi_t spi, spi_speed_t spi_speed,
                   gpio_t cs_pin, gpio_t int_pin, gpio_t reset_pin,
                   gpio_t tcxo_pin)
{
    dev->driver = &ng_ml7396_driver;

    /* initialize device descriptor */
    dev->spi = spi;
    dev->cs_pin = cs_pin;
    dev->int_pin = int_pin;
    dev->reset_pin = reset_pin;
    dev->tcxo_pin = tcxo_pin;

    mutex_init(&dev->mutex);

    /* create TX thread */
    ng_ml7396_tx_init(dev);

    /* create IRQ-BH thread */
    ng_ml7396_irq_init(dev);

    /* initialize SPI, GPIOs */
    _ng_ml7396_spi_init(dev, spi_speed);
    _ng_ml7396_gpio_interrupt_init(dev);

    //ng_ml7396_reset(dev);

#if ENABLE_DEBUG
    dev_debug = dev;
#endif

    return 0;
}

void ng_ml7396_reset(ng_ml7396_t *dev)
{
    /* ML7396 RESET */
    gpio_set(dev->reset_pin);
    usleep(20);
    gpio_clear(dev->reset_pin);
    usleep(20);
    gpio_set(dev->reset_pin);

    usleep(100 * 1000);

    _ng_ml7396_clk_init(dev);
    _ng_ml7396_rf_init(dev);

    /* set default address */
    ng_ml7396_set_addr_long(dev, NG_ML7396_DEFAULT_ADDR_LONG);
    ng_ml7396_set_addr_short(dev, NG_ML7396_DEFAULT_ADDR_SHORT);

    /* use long address default */
    ng_ml7396_set_option(dev, NG_ML7396_OPT_SRC_ADDR_LONG, true);

    /* set default PAN id */
    ng_ml7396_set_pan(dev, NG_ML7396_DEFAULT_PANID);

    /* set default channel */
    ng_ml7396_set_chan(dev, NG_ML7396_DEFAULT_CHANNEL);

    /* set default protocol */
#ifdef MODULE_NG_SIXLOWPAN
    dev->proto = NG_NETTYPE_SIXLOWPAN;
#else
    dev->proto = NG_NETTYPE_UNDEF;
#endif

    dev->max_retries = 1;

    ng_ml7396_switch_to_rx(dev);
}

void ng_ml7396_set_chan(ng_ml7396_t *dev, uint8_t channel)
{
    uint8_t ch;

    dev->chan = channel;
    ch = (uint8_t) ((channel - 33) / 2);

    ng_ml7396_reg_write(dev, ML7396_REG_CH_SET, ch);
}

bool ng_ml7396_cca(ng_ml7396_t *dev)
{
    uint8_t status;
    uint32_t intstat;
    bool result;

    ng_ml7396_set_interrupt_mask(dev, INT_CCA_COMPLETE);

    /* CCA Enable */
    ng_ml7396_reg_write(dev, ML7396_REG_CCA_CNTRL, CCA_EN);
    /* RF ON */
    ng_ml7396_switch_to_rx(dev);

    while (1) {
        intstat = ng_ml7396_get_interrupt_status(dev);

        if (intstat & INT_CCA_COMPLETE) {
            break;
        }

        usleep(10);
    }

    ng_ml7396_clear_interrupts(dev, INT_CCA_COMPLETE);

    status = ng_ml7396_reg_read(dev, ML7396_REG_CCA_CNTRL);

    if (CCA_RSLT(status) == CCA_RSLT_IDLE) {
        result = true;
    }
    else {
        result = false;
    }

    return result;
}

extern void dump_buffer(char *buf, size_t len);

size_t ng_ml7396_send(ng_ml7396_t *dev, uint8_t *data, size_t len)
{
    ng_pktsnip_t *pkt;
    msg_t msg, reply;

    /* check data length */
    if (len > NG_ML7396_MAX_PKT_LENGTH) {
        DEBUG("[ng_ml7396] Error: data to send exceeds max packet size\n");
        return 0;
    }

    pkt = ng_pktbuf_add(NULL, data, len, NG_NETTYPE_NETIF);

    msg.type = NG_NETAPI_MSG_TYPE_SND;
    msg.content.ptr = (char *) pkt;

    msg_send_receive(&msg, &reply, dev->tx_pid);

    return (int) msg.content.value;
}

size_t ng_ml7396_send_pkt(ng_ml7396_t *dev, ng_pktsnip_t *pkt)
{
    ng_pktsnip_t *snip, *snip_tmp;
    uint8_t mhr[NG_IEEE802154_MAX_HDR_LEN];
    uint16_t crc;
    size_t len, trim;
    int res;

    //printf("%s: pkt: %p, pkt->data: %p\n", __FUNCTION__, pkt, pkt->data);
    //usleep(100 * 1000);

    /* create 802.15.4 header */
    len = _make_data_frame_hdr(dev, mhr, (ng_netif_hdr_t *) pkt->data);

    if (len == 0) {
        DEBUG("[ng_ml7396] error: unable to create 802.15.4 header\n");
        len = -ENOMSG;
        goto ret;
    }

    /* check if packet (header + payload + FCS) fits into FIFO */
    snip = pkt->next;

    if ((ng_pkt_len(snip) + len + 2) > NG_ML7396_MAX_PKT_LENGTH) {
        DEBUG("[ng_ml7396] error: packet too large to be send\n");
        len = -EOVERFLOW;
        goto ret;
    }

    /* generate CRC */
    snip_tmp = snip;
    crc = crc_ccitt(0, mhr, len);
    dump_buffer(mhr, len);

    while (snip_tmp) {
        dump_buffer(snip_tmp->data, snip_tmp->size);
        crc = crc_ccitt(crc, snip_tmp->data, snip_tmp->size);
        snip_tmp = snip_tmp->next;
    }

    _ng_ml7396_backoff(dev);

    res = ng_ml7396_tx_prepare(dev);

    if (res == 0) {
        uint8_t phy_hdr[2];

        phy_hdr[0] = 0x10;
        phy_hdr[1] = ng_pkt_len(snip) + len + 2;

        /* load PHY header to FIFO */
        ng_ml7396_tx_load(dev, phy_hdr, 2);

        /* load MAC header */
        ng_ml7396_tx_load(dev, mhr, len);

        snip = pkt->next;

        /* load payload */
        while (snip) {
            ng_ml7396_tx_load(dev, snip->data, snip->size);
            len += snip->size;
            snip = snip->next;
        }

        /* load CRC */
        ng_ml7396_tx_load(dev, (uint8_t *) &crc, sizeof(crc));

        res = ng_ml7396_tx_exec(dev);
        printf("%s: tx_exec -> %d\n", __FUNCTION__, res);
    }

    if (res != 0) {
        len = -ETIMEDOUT;
    }

    ng_ml7396_switch_to_rx(dev);

ret:
    return len;
}

int ng_ml7396_tx_prepare(ng_ml7396_t *dev)
{
    int i, yield = 0;
    uint32_t status, enable;
    uint8_t reg;

    if (ng_ml7396_cca(dev) == false) {
        return -EBUSY;
    }

    for (i = 0; i < 3; i++) {
        status = ng_ml7396_get_interrupt_status(dev);
        reg = ng_ml7396_reg_read(dev, ML7396_REG_INT_PD_DATA_IND);

        if ((status & INT_SFD_DETECT) || (reg != 0)) {
            yield = 1;
        }
        else {
            yield = 0;
        }

        if ((yield == 1) || i > 0) {
            enable = ng_ml7396_get_interrupt_enable(dev);
            printf("%s: [%d] enable: 0x%08x, status: 0x%08x (valid: 0x%08x),"
                   " PD_DATA_IND: 0x%02x\n",
                   __FUNCTION__, i, (unsigned int) enable,
                   (unsigned int) status, (unsigned int) (enable & status), reg);
        }

        if (yield == 1) {
            /* re-enable interrupt */
            ng_ml7396_set_interrupt_mask(dev, ML7396_INT_ALL);
            ng_ml7396_set_interrupt_enable(dev, enable);

            usleep(50 * 1000);
            thread_yield();
        }
        else {
            break;
        }
    }

    if (ng_ml7396_tx_exec(dev) != 0) {
        printf("%s: RX -> TX timeouted...\n", __FUNCTION__);
        return -ETIMEDOUT;
    }

    return 0;
}


size_t ng_ml7396_tx_load(ng_ml7396_t *dev, uint8_t *data, size_t len)
{
    printf("%s: len: %d\n", __FUNCTION__, len);
    dump_buffer(data, len);

    ng_ml7396_fifo_writes(dev, data, len);

    return len;
}

int ng_ml7396_tx_exec(ng_ml7396_t *dev)
{
    int res;
    uint32_t status;
    uint8_t reg;

    reg = ng_ml7396_reg_read(dev, ML7396_REG_RF_STATUS);
    if (reg & STAT_TX_ON) {
        return 0;
    }

    ng_ml7396_clear_interrupts(dev, INT_RFSTAT_CHANGE);
    ng_ml7396_reg_write(dev, ML7396_REG_RF_STATUS, SET_TX_ON);

    res = _ng_ml7396_wait_rf_stat_poll(dev, STAT_TX_ON);

    status = ng_ml7396_get_interrupt_status(dev);
    status &= (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE |
               INT_TXFIFO0_REQ | INT_TXFIFO1_REQ |
               INT_RFSTAT_CHANGE);

    ng_ml7396_clear_interrupts(dev, status);
    ng_ml7396_reg_write(dev, ML7396_REG_INT_PD_DATA_REQ, 0x00);

    return res;
}

int ng_ml7396_switch_to_rx(ng_ml7396_t *dev)
{
    uint8_t reg;

    reg = ng_ml7396_reg_read(dev, ML7396_REG_RF_STATUS);
    if (reg & STAT_RX_ON) {
        return 0;
    }

    ng_ml7396_clear_interrupts(dev, INT_RFSTAT_CHANGE);
    ng_ml7396_reg_write(dev, ML7396_REG_RF_STATUS, SET_RX_ON);

    return _ng_ml7396_wait_rf_stat_poll(dev, STAT_RX_ON);
}

void ng_ml7396_rx_read(ng_ml7396_t *dev, uint8_t *data, size_t len)
{
    ng_ml7396_fifo_reads(dev, data, len);
}


/* debug functions */
#if ENABLE_DEBUG

static void _debug_dump_buffer(char *buf, size_t len)
{
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
}

#define REGADDR_MAX (0x7e)

void ng_ml7396_regdump(int bank)
{
    int i;
    char buf[REGADDR_MAX];

    if (!dev_debug) {
        return;
    }

    if (bank < 0 || bank > 2) {
        DEBUG("%s: invalid bank no. (%d)\n", __FUNCTION__, bank);
        return;
    }

    ng_ml7396_reg_reads(dev_debug, ((uint16_t) (bank | 0x80)) << 8, buf, REGADDR_MAX);

    DEBUG("-- ML7396B Bank: %d --\n", bank);
    _debug_dump_buffer(buf, REGADDR_MAX);
}
#else
void ng_ml7396_regdump(int bank)
{

}
#endif
