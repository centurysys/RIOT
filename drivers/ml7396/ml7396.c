/*
 * Copyright (C) 2015
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ml7396
 * @{
 *
 * @file
 * @brief       Driver implementation of the ML7396 device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * @}
 */

#include <strings.h>
#include <unistd.h>
#include <errno.h>

#include "thread.h"
#include "msg.h"

#include "board.h"
#include "ml7396.h"
#include "ml7396_spi.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "kernel_types.h"
#include "transceiver.h"
#include "hwtimer.h"
#include "mutex.h"
#include "ringbuffer.h"
#include "config.h"

//#define DEBUG

#define INT_WAIT 4

kernel_pid_t ml7396_irq_thread_pid = KERNEL_PID_UNDEF;
static char ml7396_irq_thread_stack[1024];


static mutex_t ml7396_mutex = MUTEX_INIT;
static mutex_t ml7396_ex = MUTEX_INIT;

//static int wait_interrupt = 0;

static uint16_t radio_pan;
static uint8_t  radio_channel;
static uint16_t radio_address;
static uint64_t radio_address_long;

netdev_802154_raw_packet_cb_t ml7396_raw_packet_cb;
netdev_rcv_data_cb_t ml7396_data_packet_cb;

/* default source address length for sending in number of byte */
static size_t _default_src_addr_len = 8;

static void ml7396_irq(void);
static void ml7396_spi_init(void);
static void ml7396_gpio_interrupt_init(void);
static void ml7396_rf_init(void);

static void *ml7396_irq_thread(void *arg);
static void ml7396_irq_bh(void);


#ifndef ML7396_SPI_SPEED
#define SPI_SPEED    SPI_SPEED_15MHZ
#else
#define SPI_SPEED    ML7396_SPI_SPEED
#endif


struct wait_interrupt {
    uint32_t int_wait;
    int clear;
    mutex_t *mutex;
};

static struct wait_interrupt int_waits[INT_WAIT];


int ml7396_wait_interrupt(uint32_t interrupts, int clear, mutex_t *mutex)
{
    int i, res = -EBUSY;
    struct wait_interrupt *wait = NULL;
    unsigned irqstate;

    irqstate = disableIRQ();

    for (i = 0; i < INT_WAIT; i++) {
        if (!int_waits[i].int_wait) {
            wait = &int_waits[i];
        }
    }

    if (!wait) {
        goto ret;
    }

    wait->int_wait = interrupts;
    wait->clear = clear;
    wait->mutex = mutex;

    restoreIRQ(irqstate);

    mutex_lock(mutex);

    irqstate = disableIRQ();

    if (wait->clear)
        ml7396_clear_interrupts(interrupts);

    wait->int_wait = 0;
    wait->mutex = NULL;
    wait->clear = 0;

    res = 0;

ret:
    restoreIRQ(irqstate);
    return res;
}

int ml7396_wakeup_interrupt(uint32_t interrupt)
{
    int i, cnt = 0;
    struct wait_interrupt *wait = NULL;
    unsigned irqstate;

    irqstate = disableIRQ();

    for (i = 0; i < INT_WAIT; i++) {
        wait = &int_waits[i];

        if ((wait->int_wait & interrupt) && (wait->mutex != NULL)) {
            mutex_unlock(wait->mutex);
            cnt++;
        }
    }

    restoreIRQ(irqstate);
    return cnt;
}

int ml7396_initialize(netdev_t *dev)
{
    kernel_pid_t pid;

    if (ml7396_irq_thread_pid == KERNEL_PID_UNDEF) {
        pid = thread_create(ml7396_irq_thread_stack,
                            sizeof(ml7396_irq_thread_stack),
                            1,
                            CREATE_STACKTEST | CREATE_SLEEPING,
                            ml7396_irq_thread,
                            NULL,
                            "ml7396_bh");
        ml7396_irq_thread_pid = pid;
        thread_wakeup(pid);
    }

    ml7396_spi_init();
    ml7396_gpio_interrupt_init();

    ml7396_reset();

    return 0;
}

#ifdef MODULE_TRANSCEIVER
void ml7396_init(kernel_pid_t tpid)
{
    transceiver_pid = tpid;
    ml7396_initialize(NULL);
}
#endif

static void ml7396_spi_init(void)
{
    int res;

    res = spi_init_master(ML7396_SPI, SPI_CONF_FIRST_RISING, SPI_SPEED);

    if (res < 0) {
        printf("%s: spi_init_master failed.\n", __FUNCTION__);
        return;
    }

    gpio_init_out(ML7396_CS, GPIO_PULLUP);
    gpio_init_out(ML7396_RESET, GPIO_PULLUP);
    gpio_init_out(ML7396_TCXO, GPIO_PULLUP);

    gpio_clear(ML7396_TCXO);
}

static void ml7396_gpio_interrupt_init(void)
{
    gpio_init_int(ML7396_INT, GPIO_PULLUP, GPIO_FALLING, (gpio_cb_t) ml7396_irq, NULL);

    ml7396_set_interrupt_mask(ML7396_INT_ALL);
}

static void ml7396_clk_init(void)
{
    //ml7396_clear_interrupts(INT_CLK_STABLE | INT_VCO_DONE);
    //ml7396_set_interrupt_enable(INT_CLK_STABLE);

    //printf("stat: 0x%08x\n", ml7396_get_interrupt_status());
    //printf("enable: 0x%08x\n", ml7396_get_interrupt_enable());

    //wait_interrupt = INT_CLK_STABLE;
    ml7396_set_interrupt_enable(INT_CLK_STABLE);

    ml7396_reg_write(ML7396_REG_RST_SET, 0xff);
    ml7396_reg_write(ML7396_REG_CLK_SET, (CLK_Done | TCXO_EN |
                                          CLK3_EN | CLK2_EN | CLK1_EN | CLK0_EN));

    ml7396_wait_interrupt(INT_CLK_STABLE, 1, &ml7396_mutex);

    //dprintf("stat: 0x%08x\n", (unsigned int) ml7396_get_interrupt_status());

    //ml7396_set_interrupt_mask(ML7396_INT_ALL);
}

struct ml7396_setting {
    uint16_t reg;
    uint8_t val;
};

static struct ml7396_setting rf_settings[] = {
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

static void ml7396_setup_rf_basic_settings(void)
{
    struct ml7396_setting *conf;

    for (conf = rf_settings; conf->reg != 0xffff; conf++) {
/*      printf("%s bank[%d] reg[0x%02x] <- 0x%02x\n",
               (conf->reg & 0x8000 ? "#" : " "),
               (conf->reg & 0x7f00) >> 8, (conf->reg & 0xfe) >> 1, conf->val);*/
        ml7396_reg_write(conf->reg, conf->val);
    }
}

static void ml7396_rf_init(void)
{
    uint8_t val;
    int16_t bpf_offset, bpf_cca_offset, sign;
    uint16_t bpf_val, bpf_cca_val;

    ml7396_setup_rf_basic_settings();

    /* get BPF_ADJ_OFFSET */
    val = ml7396_reg_read(ML7396_REG_BPF_ADJ_OFFSET);

    bpf_offset = (int16_t) (val & 0x7f);
    bpf_cca_offset = (int16_t) (((int16_t) (val & 0x7f)) * 48 / 100);

    sign = (val & 0x80) ? 1 : -1;

    bpf_val = 0x024a + (bpf_offset * sign);
    bpf_cca_val = 0x0119 + (bpf_cca_offset * sign);

    /* BPF adjust */
    ml7396_reg_write(ML7396_REG_BPF_ADJ_H, (bpf_val & 0xff00) >> 8);
    ml7396_reg_write(ML7396_REG_BPF_ADJ_L, (bpf_val & 0x00ff));

    ml7396_reg_write(ML7396_REG_BPF_AFC_ADJ_H, (bpf_val & 0xff00) >> 8);
    ml7396_reg_write(ML7396_REG_BPF_AFC_ADJ_L, (bpf_val & 0x00ff));

    ml7396_reg_write(ML7396_REG_BPF_CCA_ADJ_H, (bpf_cca_val & 0xff00) >> 8);
    ml7396_reg_write(ML7396_REG_BPF_CCA_ADJ_L, (bpf_cca_val & 0x00ff));

    //wait_interrupt = INT_VCO_DONE;
    ml7396_set_interrupt_enable(INT_VCO_DONE);

    /* start VCO calibration */
    ml7396_reg_write(ML7396_REG_VCO_CAL_START, 0x01);

    /* wait for calibration done. */
    //mutex_lock(&ml7396_mutex);
    ml7396_wait_interrupt(INT_VCO_DONE, 1, &ml7396_mutex);

    //printf("VCO calibration done.\n");
    ml7396_set_interrupt_mask(INT_VCO_DONE);
}

void ml7396_reset(void)
{
    mutex_init(&ml7396_mutex);
    mutex_lock(&ml7396_mutex);

    /* ML7396 RESET */
    gpio_set(ML7396_RESET);
    usleep(1);
    gpio_clear(ML7396_RESET);
    usleep(1);
    gpio_set(ML7396_RESET);

    usleep(100 * 1000);

    ml7396_clk_init();
    ml7396_rf_init();

    ml7396_set_channel(33);

    ml7396_switch_to_rx();
}

/*
 * ML7396 GPIO IRQ handler
 */
static void ml7396_irq(void)
{
    msg_t msg;

    msg.type = MSG_ML7396_IRQ;
    msg_send_int(&msg, ml7396_irq_thread_pid);
}

/*
 * ML7396 GPIO IRQ handle bottom-half
 */
static void ml7396_irq_bh(void)
{
    int page;
    uint32_t status, enable;

    enable = ml7396_get_interrupt_enable();
    status = ml7396_get_interrupt_status();

    /* FIFO 0/1 RX Done/CRCError interrupt */
    if (status & (INT_RXFIFO0_DONE | INT_RXFIFO1_DONE |
                  INT_RXFIFO0_CRCERR | INT_RXFIFO1_CRCERR |
                  INT_RXFIFO_ERR)) {
        page = ml7396_rx_handler(status);

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
        ml7396_wakeup_interrupt(status);

        enable &= ~status;
    }

    enable |= (INT_RXFIFO1_CRCERR | INT_RXFIFO0_CRCERR |
               INT_RXFIFO1_DONE | INT_RXFIFO0_DONE |
               INT_RXFIFO_ERR);

    /* re-enable interrupt */
    ml7396_set_interrupt_mask(ML7396_INT_ALL);
    ml7396_set_interrupt_enable(enable);
}

int ml7396_add_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb)
{
    if (ml7396_raw_packet_cb == NULL){
        ml7396_raw_packet_cb = recv_cb;
        return 0;
    }

    return -ENOBUFS;
}

int ml7396_rem_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb)
{
    ml7396_raw_packet_cb = NULL;
    return 0;
}

int ml7396_add_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb)
{
    if (ml7396_data_packet_cb == NULL){
        ml7396_data_packet_cb = recv_cb;
        return 0;
    }

    return -ENOBUFS;
}

int ml7396_rem_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb)
{
    ml7396_data_packet_cb = NULL;
    return 0;
}

int ml7396_channel_is_clear(netdev_t *dev)
{
    uint8_t status;
    uint32_t intstat;
    int result;

    //ml7396_set_interrupt_enable(INT_RFSTAT_CHANGE | INT_CCA_COMPLETE);
    ml7396_set_interrupt_mask(INT_CCA_COMPLETE);

    /* CCA Enable */
    ml7396_reg_write(ML7396_REG_CCA_CNTRL, CCA_EN);
    /* RF ON */
    ml7396_switch_to_rx();

    //ml7396_wait_interrupt(INT_CCA_COMPLETE, 1, &ml7396_mutex);

    while (1) {
        intstat = ml7396_get_interrupt_status();

        if (intstat & INT_CCA_COMPLETE) {
            break;
        }

        usleep(20);
    }

    ml7396_clear_interrupts(INT_CCA_COMPLETE);

    status = ml7396_reg_read(ML7396_REG_CCA_CNTRL);

    if (CCA_RSLT(status) == CCA_RSLT_IDLE) {
        result = 0;
    }
    else {
        result = -1;
    }

    return result;
}


uint16_t ml7396_set_address(uint16_t address)
{
    radio_address = address;

    return radio_address;
}

radio_address_t ml7396_get_address(void)
{
    return radio_address;
}

static void _ml7396_set_address_filter_long(void)
{
    int i;
    uint8_t val[8];

    for (i = 0; i < 8; i++) {
        val[i] = (uint8_t) ((radio_address_long >> (8 * i)) & 0xff);
    }

    ml7396_reg_writes(ML7396_REG_64ADDR1, val, 8);
}

uint64_t ml7396_set_address_long(uint64_t address)
{
    radio_address_long = address;
    _ml7396_set_address_filter_long();

    return radio_address_long;
}

uint64_t ml7396_get_address_long(void)
{
    return radio_address_long;
}

void ml7396_get_address_long_buf(uint8_t *buf)
{
    uint64_t addr;
    int i;

    addr = ml7396_get_address_long();
    for (i = 0; i < 8; i++) {
        buf[i] = (uint8_t) ((addr >> (8 * (7 - i))) & 0xff);
    }
}

static void _ml7396_set_pan_filter(void)
{
    int i;
    uint8_t val[2];

    for (i = 0; i < 2; i++) {
        val[i] = (uint8_t) ((radio_pan >> (8 * i)) & 0xff);
    }

    ml7396_reg_writes(ML7396_REG_PANID_L, val, 2);
}

uint16_t ml7396_set_pan(uint16_t pan)
{
    radio_pan = pan;
    _ml7396_set_pan_filter();

    return radio_pan;
}

uint16_t ml7396_get_pan(void)
{
    return radio_pan;
}

static void _ml7396_set_channel(unsigned int channel)
{
    uint8_t ch;

    ch = (uint8_t) ((channel - 33) / 2);

    ml7396_reg_write(ML7396_REG_CH_SET, ch);
}

/*
 *
 */
int ml7396_set_channel(unsigned int channel)
{
    if (channel < ML7396_MIN_CHANNEL ||
        channel > ML7396_MAX_CHANNEL ||
        (channel % 2) == 0) {
#if DEVELHELP
        puts("[ml7396] channel out of range!");
#endif
        return -1;
    }

    radio_channel = channel;

    _ml7396_set_channel(radio_channel);

    return radio_channel;
}

unsigned int ml7396_get_channel(void)
{
    return radio_channel;
}

int ml7396_get_option(netdev_t *dev, netdev_opt_t opt, void *value,
                      size_t *value_len)
{
    if (dev != &ml7396_netdev) {
        return -ENODEV;
    }

    switch (opt) {
        case NETDEV_OPT_CHANNEL:
            if (*value_len < sizeof(unsigned int)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(unsigned int)) {
                *value_len = sizeof(unsigned int);
            }
            *((unsigned int *) value) = ml7396_get_channel();
            break;

        case NETDEV_OPT_NID:
            if (*value_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint16_t)) {
                *value_len = sizeof(uint16_t);
            }
            *((uint16_t *) value) = ml7396_get_pan();
            break;

        case NETDEV_OPT_ADDRESS_LONG:
            if (*value_len < sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint64_t)) {
                *value_len = sizeof(uint64_t);
            }
            *((uint64_t *) value) = ml7396_get_address_long();
            break;

        case NETDEV_OPT_MAX_PACKET_SIZE:
            if (*value_len == 0) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint8_t)) {
                *value_len = sizeof(uint8_t);
            }
            *((uint8_t *) value) = ML7396_MAX_PKT_LENGTH;
            break;

        case NETDEV_OPT_PROTO:
            if (*value_len < sizeof(netdev_proto_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(netdev_proto_t)) {
                *value_len = sizeof(netdev_proto_t);
            }
            *((netdev_type_t *) value) = NETDEV_PROTO_802154;
            break;

        case NETDEV_OPT_SRC_LEN:
            if (*value_len < sizeof(size_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(size_t)) {
                *value_len = sizeof(size_t);
            }
            *((size_t *) value) = _default_src_addr_len;

        default:
            return -ENOTSUP;
    }

    return 0;
}


int ml7396_set_option(netdev_t *dev, netdev_opt_t opt, void *value,
                      size_t value_len)
{
    if (dev != &ml7396_netdev) {
        return -ENODEV;
    }

    switch (opt) {
        case NETDEV_OPT_CHANNEL:
            if (value_len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            if (ml7396_set_channel(*((uint8_t *) value)) == -1) {
                return -EINVAL;
            }
            break;

        case NETDEV_OPT_NID:
            if (value_len != sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            else {
                uint16_t *v = (uint16_t *) value;

                if (*v == 0xffff) {
                    /* Do not allow setting to broadcast */
                    return -EINVAL;
                }
                ml7396_set_pan(*v);
            }
            break;

        case NETDEV_OPT_ADDRESS_LONG:
            if (value_len != sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            else {
                uint64_t *v = (uint64_t *) value;
                ml7396_set_address_long(*v);
            }
            break;

        case NETDEV_OPT_SRC_LEN:
            if (value_len != sizeof(size_t)) {
                return -EOVERFLOW;
            }
            else {
                size_t *v = (size_t *) value;

                if (*v != 8) {
                    return -EINVAL;
                }
                _default_src_addr_len = *v;
            }
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}

int ml7396_get_state(netdev_t *dev, netdev_state_t *state)
{


    return 0;
}

int ml7396_set_state(netdev_t *dev, netdev_state_t state)
{

    return 0;
}

void ml7396_lock(void)
{
    mutex_lock(&ml7396_ex);
}

void ml7396_unlock(void)
{
    mutex_unlock(&ml7396_ex);
}


static void __attribute__((unused)) _ml7396_wait_rf_stat(uint8_t stat, const char *func)
{
    volatile uint8_t reg;

    while (1) {
        reg = ml7396_reg_read(ML7396_REG_RF_STATUS);
        /*printf("# %s: RF_STATUS = 0x%02x, (wait: 0x%02x) (%s)\n",
          __FUNCTION__, reg, stat, func);*/

        if ((reg & 0xf0) == stat) {
            break;
        }
#if 1
        else {
            ml7396_clear_interrupts(INT_RFSTAT_CHANGE);
            ml7396_set_interrupt_enable(INT_RFSTAT_CHANGE);
            ml7396_wait_interrupt(INT_RFSTAT_CHANGE, 0, &ml7396_mutex);
        }
#endif
    }

    //printf("%s: done. (%s)\n", __FUNCTION__, func);
}

void ml7396_switch_to_rx(void)
{
    ml7396_reg_write(ML7396_REG_RF_STATUS, SET_RX_ON);

    //_ml7396_wait_rf_stat(STAT_RX_ON, __FUNCTION__);
}

void ml7396_switch_to_tx(void)
{
    ml7396_reg_write(ML7396_REG_RF_STATUS, SET_TX_ON);

    //_ml7396_wait_rf_stat(STAT_TX_ON, __FUNCTION__);
}

int ml7396_switch_to_trx_off(void)
{
    int i, res;
    volatile uint8_t reg;

    ml7396_reg_write(ML7396_REG_RF_STATUS, SET_TRX_OFF);
    //_ml7396_wait_rf_stat(STAT_TRX_OFF, __FUNCTION__);

    res = -1;
    for (i = 0; i < 10; i++) {
        reg = ml7396_reg_read(ML7396_REG_RF_STATUS);

        if ((reg & 0xf0) == STAT_TRX_OFF) {
            res = 0;
            break;
        }
        usleep(1 * 1000);
    }

    return res;
}

/*
 * IRQ handle bottom-half
 */
static void *ml7396_irq_thread(void *arg)
{
    int res;
    msg_t msg;
    timex_t timeout;

    while (1) {
        timeout.seconds = 2;
        timeout.microseconds = 0;

        res = vtimer_msg_receive_timeout(&msg, timeout);

        if (res == 1) {
            switch (msg.type) {
                case MSG_ML7396_IRQ:
                    ml7396_irq_bh();
                    break;

                default:
                    break;
            }
        }
        else {
            /* polling */
            ml7396_irq_bh();
        }
    }

    return NULL;
}


const netdev_802154_driver_t ml7396_driver = {
    .init = ml7396_initialize,
//  .send_data = netdev_802154_send_data,
    .add_receive_data_callback = ml7396_add_data_recv_callback,
    .rem_receive_data_callback = ml7396_rem_data_recv_callback,
    .get_option = ml7396_get_option,
    .set_option = ml7396_set_option,
    .get_state = ml7396_get_state,
    .set_state = ml7396_set_state,
//  .event = ml7396_event,
//  .load_tx = ml7396_load_tx_buf,
//  .transmit = ml7396_transmit_tx_buf,
//  .send = netdev_802154_send,
    .add_receive_raw_callback = ml7396_add_raw_recv_callback,
    .rem_receive_raw_callback = ml7396_rem_raw_recv_callback,
    .channel_is_clear = ml7396_channel_is_clear,
};

netdev_t ml7396_netdev = { NETDEV_TYPE_802154, (netdev_driver_t *) &ml7396_driver, NULL };
