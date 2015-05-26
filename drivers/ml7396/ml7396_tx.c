#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "periph/random.h"

#include "mutex.h"
#include "ieee802154_frame.h"

#include "ml7396.h"
#include "ml7396_spi.h"
#include "crc.h"

/* CSMA parameters */
static const uint32_t aUnitBackoffPeriod = 1130;
static const int macMaxCSMABackoffs = 4;
static const int macMinBE = 8;
static const int macMaxBE = 8;
//static const int macMaxFrameRetries = 3;


static int16_t ml7396_load(ml7396_packet_t *packet);
static int16_t ml7396_load_raw(char *buf, int len);
static void ml7396_gen_pkt(uint8_t *buf, ml7396_packet_t *packet);

static uint8_t sequence_nr;

/* FCS is added in hardware */
static uint8_t pkt[256 + 2];

static ml7396_packet_t ml7396_tx_packet;


int16_t ml7396_send(ml7396_packet_t *packet)
{
    int16_t result;
    int retry, status;

    ml7396_lock();

#if 1
    /* CCA */
    for (retry = 0; retry < 3; retry++) {
        status = ml7396_channel_is_clear(&ml7396_netdev);

        if (status == 0) {
            break;
        }
        else {
            usleep(1 * 1000);
        }
    }

    if (status != 0) {
        result = -1;
        goto ret;
    }
#endif

    result = ml7396_load(packet);
    if (result < 0) {
        goto ret;
    }

    ml7396_transmit_tx_buf(&ml7396_netdev);

ret:
    ml7396_unlock();
    return result;
}

extern void uart1_rx_set(int stat);

static int send_ack_req = 0;

int16_t ml7396_send_raw(char *buf, int len)
{
    int16_t result;
    int i, retry, status;
    uint32_t backoff;
    uint8_t rand;

    ieee802154_frame_read((uint8_t *) buf, &ml7396_tx_packet.frame, len);

    if (ml7396_tx_packet.frame.fcf.ack_req == 1) {
        /* unicast */
        retry = 4;
    }
    else {
        retry = 1;
    }

retry:
    result = 0;

    /* CCA */
    for (i = 0; i < retry; i++) {
        random_read((char *) &rand, 1);
        backoff = aUnitBackoffPeriod * rand; /* usecs */

        uart1_rx_set(101);

        usleep(backoff);

        uart1_rx_set(102);

        /* --- Begin ML7396 critical section --- */
        ml7396_lock();

        status = ml7396_channel_is_clear(&ml7396_netdev);

        uart1_rx_set(103);

        if (status == 0) {
            uart1_rx_set(104);

            result = ml7396_load_raw(buf, len);

            if (result > 0) {
                break;
            }
        }
        else if (send_ack_req == 1) {
            send_ack_req = 0;
            ml7396_unlock();
            printf("%s: received in CCA[%d], retry.\n", __FUNCTION__, i);
            goto retry;
        }

        ml7396_unlock();
    }

    if (result > 0) {
        uart1_rx_set(106);

        ml7396_transmit_tx_buf(&ml7396_netdev);
        result = 0;

        uart1_rx_set(107);

        /* --- End ML7396 critical section --- */
        ml7396_unlock();
    }

    uart1_rx_set(108);

    return result;
}

int16_t ml7396_send_ack(ieee802154_frame_t *frame, int enhanced)
{
    ml7396_packet_t packet;
    int i, done;
    uint8_t reg;
    int16_t result;
    uint32_t status;

    send_ack_req = 1;

    /* --- BEGIN ML7396 critical section --- */
    ml7396_lock();

    if (ml7396_switch_to_trx_off() != 0) {
        printf("%s: TRX_OFF timeouted...\n", __FUNCTION__);
        result = -1;
        goto ret;
    }

    memset(&packet, 0, sizeof(ml7396_packet_t));
    //printf("%s: ACK seq_nr = 0x%02x\n", __FUNCTION__, frame->seq_nr);

    packet.frame.seq_nr = frame->seq_nr;
    packet.frame.src_pan_id = ml7396_get_pan();

    if (frame->fcf.panid_comp == 1) {
        packet.frame.dest_pan_id = frame->dest_pan_id;
    }
    else {
        packet.frame.dest_pan_id = frame->src_pan_id;
    }

    memcpy(packet.frame.dest_addr, frame->src_addr, 8);

    if (frame->src_pan_id == frame->dest_pan_id) {
        packet.frame.fcf.panid_comp = 1;
    }
    else {
        packet.frame.fcf.panid_comp = 0;
    }

    packet.frame.fcf.frame_type = IEEE_802154_ACK_FRAME;
    packet.frame.fcf.frame_ver = frame->fcf.frame_ver;
    packet.frame.fcf.src_addr_m = 0;
    packet.frame.fcf.dest_addr_m = IEEE_802154_LONG_ADDR_M;

    /* calculate size of the frame (payload) */
    packet.length = ieee802154_frame_get_hdr_len(&packet.frame) + 2;

    if (packet.length > (ML7396_MAX_PKT_LENGTH)) {
        printf("%s: packet length(%d) too long.\n",
               __FUNCTION__, packet.length);
        result = -1;
        goto ret;
    }

    ml7396_gen_pkt(pkt, &packet);

    reg = ml7396_reg_read(ML7396_REG_INT_PD_DATA_REQ);
    if (reg & (PD_DATA_REQ1 | PD_DATA_REQ0)) {
        printf("%s: PD_DATA_REQ is set, 0x%02x\n", __FUNCTION__, reg);
        ml7396_reg_write(ML7396_REG_INT_PD_DATA_REQ, 0x00);
    }

    ml7396_set_interrupt_enable(INT_TXFIFO0_REQ | INT_TXFIFO1_REQ);
    ml7396_fifo_write(pkt, packet.length + 4);

    done = 0;
    for (i = 0; i < 100; i++) {
        status = ml7396_get_interrupt_status();

        if (status & (INT_TXFIFO0_REQ | INT_TXFIFO1_REQ)) {
            done = 1;
            break;
        }
    }

    if (done == 1) {
        ml7396_transmit_tx_buf(&ml7396_netdev);
        result = 0;
    }
    else {
        printf("%s: TX-FIFO timeouted...\n", __FUNCTION__);
        result = -1;
    }

ret:
    send_ack_req = 0;

    /* --- End ML7396 critical section --- */
    ml7396_unlock();
    return result;
}

netdev_802154_tx_status_t ml7396_transmit_tx_buf(netdev_t *dev)
{
    uint32_t status;

    ml7396_switch_to_tx();

    while (1) {
        status = ml7396_get_interrupt_status();

        if (status & (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE))
            break;
    }

    status = ml7396_get_interrupt_status();
    status &= (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE |
               INT_TXFIFO0_REQ | INT_TXFIFO1_REQ |
               INT_RFSTAT_CHANGE);

    ml7396_clear_interrupts(status);

    status = ml7396_get_interrupt_status();

    ml7396_switch_to_rx();

    return NETDEV_802154_TX_STATUS_OK;
}

static int16_t ml7396_load(ml7396_packet_t *packet)
{
    int i, done;
    uint32_t status;

    /* RX on 状態だと、TX FIFO に書き込んだ最後の2octetsの書き込みが無視される */
    /*  --> CRC機能をしようしなければ問題ない模様 */
    ml7396_switch_to_trx_off();

    packet->frame.fcf.frame_ver = 0b00;
    packet->frame.src_pan_id = ml7396_get_pan();

    if (packet->frame.src_pan_id == packet->frame.dest_pan_id) {
        packet->frame.fcf.panid_comp = 1;
    }
    else {
        packet->frame.fcf.panid_comp = 0;
    }

    if (packet->frame.fcf.src_addr_m == 2) {
        uint16_t addr = ml7396_get_address();

        packet->frame.src_addr[0] = (uint8_t)(addr & 0xFF);
        packet->frame.src_addr[1] = (uint8_t)(addr >> 8);
    }
    else if (packet->frame.fcf.src_addr_m == 3) {
        uint64_t addr_long = ml7396_get_address_long();

        packet->frame.src_addr[0] = (uint8_t)(addr_long & 0xff);
        packet->frame.src_addr[1] = (uint8_t)(addr_long >> (1 * 8));
        packet->frame.src_addr[2] = (uint8_t)(addr_long >> (2 * 8));
        packet->frame.src_addr[3] = (uint8_t)(addr_long >> (3 * 8));
        packet->frame.src_addr[4] = (uint8_t)(addr_long >> (4 * 8));
        packet->frame.src_addr[5] = (uint8_t)(addr_long >> (5 * 8));
        packet->frame.src_addr[6] = (uint8_t)(addr_long >> (6 * 8));
        packet->frame.src_addr[7] = (uint8_t)(addr_long >> (7 * 8));
    }

    packet->frame.seq_nr = sequence_nr++;

    /* calculate size of the frame (payload) */
    packet->length = ieee802154_frame_get_hdr_len(&packet->frame) +
                     packet->frame.payload_len + 2;

    if (packet->length > (ML7396_MAX_PKT_LENGTH)) {
        return -1;
    }

    /* generate pkt */
    ml7396_gen_pkt(pkt, packet);

    ml7396_set_interrupt_enable(INT_TXFIFO0_REQ | INT_TXFIFO1_REQ);

    //printf("+++ packet len: %d octets +++\n", packet->length);
#if 0
    for (i = 0; i < packet->length; i++) {
        printf(" %02x", pkt[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

    ml7396_fifo_write(pkt, packet->length + 2);

    done = 0;
    for (i = 0; i < 100; i++) {
        status = ml7396_get_interrupt_status();

        if (status & (INT_TXFIFO0_REQ | INT_TXFIFO1_REQ)) {
            done = 1;
            break;
        }
    }

    if (done == 1) {
        //dprintf(" ---> TX-FIFO request accepted.\n");
    }
    else {
        printf("%s: TX-FIFO timeouted...\n", __FUNCTION__);
        return -1;
    }

    return packet->length;
}

static int16_t ml7396_load_raw(char *buf, int len)
{
    int i, done;
    uint32_t status;

    if (len < 0 || len > 255) {
        printf("! %s: length err (%d)\n", __FUNCTION__, len);
        return -1;
    }

    if (ml7396_switch_to_trx_off() != 0) {
        printf("%s: TRX_OFF timeouted...\n", __FUNCTION__);
        return -1;
    }

    memset(pkt, 0, 256 + 2);

    pkt[0] = 0x10;
    pkt[1] = len;

    memcpy(&pkt[2], buf, len);

    ml7396_fifo_write((uint8_t *) pkt, len + 2);

    done = 0;
    for (i = 0; i < 10; i++) {
        status = ml7396_get_interrupt_status();

        if (status & (INT_TXFIFO0_REQ | INT_TXFIFO1_REQ)) {
            done = 1;
            break;
        }

        usleep(100);
    }

    if (done != 1) {
        printf("%s: TX-FIFO timeouted...\n", __FUNCTION__);
        return -1;
    }

    return len + 2;
}

/**
 * @brief Static function to generate byte array from at86rf231 packet.
 */
static void ml7396_gen_pkt(uint8_t *buf, ml7396_packet_t *packet)
{
    uint8_t index, offset;
    uint16_t crc;

    memset(buf, 0, 256);

    buf[0] = 0x10;
    buf[1] = packet->length + 2;

    index = ieee802154_frame_init(&packet->frame, &buf[2]) + 2;
    offset = index;

    while (index < packet->length + 2) {
        buf[index] = packet->frame.payload[index - offset];
        index += 1;
    }

    crc = crc_ccitt(0, &buf[2], packet->length);
    buf[index++] = (uint8_t) (crc & 0xff);
    buf[index++] = (uint8_t) ((crc >> 8) & 0xff);
}
