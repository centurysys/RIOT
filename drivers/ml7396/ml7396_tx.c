#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "mutex.h"
#include "ieee802154_frame.h"

#include "ml7396.h"
#include "ml7396_spi.h"



static int16_t ml7396_load(ml7396_packet_t *packet);
static int16_t ml7396_load_raw(char *buf, int len);
static int16_t ml7396_load_raw2(char *buf, int len);
static void ml7396_gen_pkt(uint8_t *buf, ml7396_packet_t *packet);

static uint8_t sequence_nr;
static uint8_t wait_for_ack;

/* FCS is added in hardware */
static uint8_t pkt[256];

static ml7396_packet_t ml7396_tx_packet;


int16_t ml7396_send(ml7396_packet_t *packet)
{
    int16_t result;
    int retry, status;

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
        return -1;
    }
#endif

    result = ml7396_load(packet);
    if (result < 0) {
        return result;
    }

    ml7396_transmit_tx_buf(&ml7396_netdev);

    return result;
}

int16_t ml7396_send_raw(char *buf, int len)
{
    int16_t result;
    int i, retry, status, wait_ack;

    ieee802154_frame_read(buf, &ml7396_tx_packet.frame, len);
    //printf("%s: ack_req: %d\n", __FUNCTION__, ml7396_tx_packet.frame.fcf.ack_req);

    /* CCA */
    for (retry = 0; retry < 3; retry++) {
        status = ml7396_channel_is_clear(&ml7396_netdev);

        if (status == 0) {
            break;
        }
        else {
            usleep(10 * 1000);
        }
    }

    if (status != 0) {
        return -1;
    }

    ml7396_lock();

    result = ml7396_load_raw(buf, len);
    if (result < 0) {
        goto ret;
    }

    ml7396_transmit_tx_buf(&ml7396_netdev);
    result = 0;

ret:
    ml7396_unlock();
    return result;
}

int16_t ml7396_send_raw2(char *buf, int len)
{
    int page;
    int16_t result;
    int retry, status;
    uint32_t intstat;

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
        return -1;
    }
#endif

    //ml7396_reg_write(ML7396_REG_FAST_TX_SET, 0x08);
    ml7396_switch_to_tx();

    result = ml7396_load_raw2(buf, len);
    if (result < 0) {
        return result;
    }

    while (1) {
        status = ml7396_get_interrupt_status();
        //ml7396_wait_interrupt(INT_TXFIFO0_DONE | INT_TXFIFO1_DONE, 0, &tx_mutex);

        if (status & (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE))
            break;
    }

    //printf("**** %s: wakeup! \n", __FUNCTION__);

    intstat = ml7396_get_interrupt_status();
    intstat &= (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE |
                INT_TXFIFO0_REQ | INT_TXFIFO1_REQ |
                INT_RFSTAT_CHANGE);

    if (intstat & INT_TXFIFO0_DONE) {
        page = 0;
    }
    else {
        page = 1;
    }

    //dprintf(" clear interrupt with 0x%08x\n", (unsigned int) status);
    ml7396_clear_interrupts(intstat);

    return result;
}

int16_t ml7396_send_ack(ieee802154_frame_t *frame, int enhanced)
{
    ml7396_packet_t packet;
    int i, done;
    uint8_t reg;
    int16_t result;
    uint32_t status;

    //printf("*** %s ***\n", __FUNCTION__);

    memset(&packet, 0, sizeof(ml7396_packet_t));

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
    //printf("packet.length = %d\n", packet.length);

    if (packet.length > (ML7396_MAX_PKT_LENGTH)) {
        return -1;
    }

    ml7396_gen_pkt(pkt, &packet);

#if 0
    for (i = 0; i < packet.length + 2 + 2; i++) {
        printf(" %02x", pkt[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

    ml7396_lock();

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
        //dprintf(" ---> TX-FIFO request accepted.\n");
        ml7396_transmit_tx_buf(&ml7396_netdev);
        result = 0;
    }
    else {
        printf("%s: TX-FIFO timeouted...\n", __FUNCTION__);
        result = -1;
    }

    ml7396_unlock();
    return result;
}

netdev_802154_tx_status_t ml7396_transmit_tx_buf(netdev_t *dev)
{
    int page;
    mutex_t tx_mutex = MUTEX_INIT;
    uint32_t status;
    uint8_t reg;

    reg = ml7396_reg_read(ML7396_REG_FIFO_BANK);
    //dprintf("FIFO_BANK: 0x%02x\n", reg);

//    ml7396_set_interrupt_enable(INT_TXFIFO0_DONE | INT_TXFIFO1_DONE);

    ml7396_switch_to_tx();

//    mutex_lock(&tx_mutex);

    while (1) {
        status = ml7396_get_interrupt_status();
        //ml7396_wait_interrupt(INT_TXFIFO0_DONE | INT_TXFIFO1_DONE, 0, &tx_mutex);

        if (status & (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE))
            break;
    }

    reg = ml7396_reg_read(ML7396_REG_FIFO_BANK);

    status = ml7396_get_interrupt_status();
    status &= (INT_TXFIFO0_DONE | INT_TXFIFO1_DONE |
               INT_TXFIFO0_REQ | INT_TXFIFO1_REQ |
               INT_RFSTAT_CHANGE);

    if (status & INT_TXFIFO0_DONE) {
        page = 0;
    }
    else {
        page = 1;
    }

    //dprintf(" clear interrupt with 0x%08x\n", (unsigned int) status);
    ml7396_clear_interrupts(status);

    status = ml7396_get_interrupt_status();
    //dprintf(" --> interrupt status: 0x%08x\n", (unsigned int) status);

#if 0
    reg = (page == 0) ? (PD_DATA_REQ0 | PD_DATA_CFM0) : (PD_DATA_REQ1 | PD_DATA_CFM1);
    ml7396_reg_write(ML7396_REG_INT_PD_DATA_REQ, ~reg);
#endif
    reg = ml7396_reg_read(ML7396_REG_INT_PD_DATA_REQ);
    //dprintf(" PD_DATA_REQ: 0x%02x\n", reg);

    //ml7396_switch_to_trx_off();

#if 0
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP3, 0x00);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP2, 0x00);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP2, 0x00);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP3, 0x00);

    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP1, 0xfd);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP2, 0xff);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP3, 0xff);
    ml7396_reg_write(ML7396_REG_INT_SOURCE_GRP4, 0x03);
#endif

    ml7396_switch_to_rx();

    return NETDEV_802154_TX_STATUS_OK;
}

static int16_t ml7396_load(ml7396_packet_t *packet)
{
    int i, done;
    uint32_t status;
    uint8_t reg;

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
    reg = ml7396_reg_read(ML7396_REG_FIFO_BANK);
    //dprintf("FIFO_BANK: 0x%02x\n", reg);

    /* load packet into fifo */
    //ml7396_phy_reset();
    ml7396_fifo_write(pkt, packet->length + 2);

    reg = ml7396_reg_read(ML7396_REG_FIFO_BANK);
    //dprintf("FIFO_BANK: 0x%02x\n", reg);

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
    uint8_t reg;

    //ml7396_switch_to_trx_off();

    memset(pkt, 0, 256);

    pkt[0] = 0x10;
    pkt[1] = len;

    memcpy(&pkt[2], buf, len);

    ml7396_fifo_write((uint8_t *) pkt, len + 2);

    reg = ml7396_reg_read(ML7396_REG_FIFO_BANK);
    //dprintf("FIFO_BANK: 0x%02x\n", reg);

    done = 0;
    for (i = 0; i < 100; i++) {
        status = ml7396_get_interrupt_status();

        if (status & (INT_TXFIFO0_REQ | INT_TXFIFO1_REQ)) {
            done = 1;
            break;
        }
    }

    if (done == 1) {
#if 0
        //dprintf(" ---> TX-FIFO request accepted.\n");
        for (i = 0; i < len + 2; i++) {
            printf(" %02x", pkt[i]);
            if ((i % 16) == 15)
                puts("");
        }
        puts("");
#endif
    }
    else {
        printf("%s: TX-FIFO timeouted...\n", __FUNCTION__);
        return -1;
    }

    return len + 2;
}

static int16_t ml7396_load_raw2(char *buf, int len)
{
    int i, done;
    uint32_t status;
    uint8_t reg;

    memset(pkt, 0, 256);

    pkt[0] = 0x10;
    pkt[1] = len + 2;

    memcpy(&pkt[2], buf, len);

    ml7396_fifo_write((uint8_t *) pkt, len + 2 + 2);

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
