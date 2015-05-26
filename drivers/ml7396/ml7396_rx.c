#include <stdio.h>
#include <string.h>

#include "byteorder.h"
#include "kernel_types.h"
#include "transceiver.h"
#include "msg.h"

#include "ieee802154_frame.h"

#include "ml7396.h"
#include "ml7396_spi.h"
#include "crc.h"

ml7396_packet_t ml7396_rx_buffer[ML7396_RX_BUF_SIZE];
static uint8_t buffer[ML7396_RX_BUF_SIZE][ML7396_MAX_PKT_LENGTH];
volatile uint8_t rx_buffer_next;
extern netdev_802154_raw_packet_cb_t ml7396_raw_packet_cb;


static inline void _ml7396_clear_rx_interrupts(int page, int clear_fifo)
{
    uint32_t interrupts = 0, status;

    status = ml7396_get_interrupt_status();

    if (status & INT_RXFIFO_ERR) {
        puts("! RXFIFO_ERR -> clear_fifo");
        interrupts |= INT_RXFIFO_ERR;
    }

    if (page == 0) {
        interrupts |= (INT_RXFIFO0_DONE | INT_RXFIFO0_CRCERR);
        if ((clear_fifo == 1) && (status & INT_FIFO_CLR0)) {
            //interrupts |= INT_FIFO_CLR0;
        }
    }
    else {
        interrupts |= (INT_RXFIFO1_DONE | INT_RXFIFO1_CRCERR);
        if ((clear_fifo == 1) && (status & INT_FIFO_CLR1)) {
            //interrupts |= INT_FIFO_CLR1;
        }
    }

    if (clear_fifo == 1) {
        printf("*** clear with: 0x%08x (status: 0x%08x)\n",
               (unsigned int) interrupts, (unsigned int) status);
        ml7396_phy_reset();
    }

    ml7396_clear_interrupts(interrupts);
    //ml7396_reg_write(ML7396_REG_INT_PD_DATA_IND, 0x00);

    if (clear_fifo == 1) {
        status = ml7396_get_interrupt_status();
        printf(" --> after (status: 0x%08x)\n", (unsigned int) status);
    }
}

static void _send_ack(ieee802154_frame_t *frame)
{
    uint16_t pan_id;
    uint8_t addr[8];

    pan_id = ml7396_get_pan();
    ml7396_get_address_long_buf(addr);

    if ((frame->fcf.ack_req == 1) &&
        (pan_id == frame->dest_pan_id) &&
        (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
        memcmp(frame->dest_addr, addr, 8) == 0) {
        ml7396_send_ack(frame, /* enhanced = */ 1);

        //printf("%s: send ACK\n", __FUNCTION__);
    }
}

int ml7396_rx_handler(uint32_t status)
{
    int phr_len, page, complete, crc_ok, clear_fifo;
    int int_pend[2] = {0, 0};
    uint8_t *buf, lqi, fcs_rssi;
    uint16_t tmp, crc, crc_received;
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
        printf("%s: page ???\n", __FUNCTION__);
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
    ml7396_lock();

    /* read PHR (2-octets) */
    ml7396_fifo_read((uint8_t *) &tmp, 2);

    phr_len = (int) (NTOHS(tmp) & 0xff) - 2;

    if (phr_len < 10) {
        printf("%s: ??? phr_len = %d, intstat = 0x%08x, page = %d\n",
               __FUNCTION__, phr_len, (unsigned int) status, page);
        clear_fifo = 1;

        ml7396_unlock();

        goto ret;
    }

    /* read psdu */
    buf = buffer[rx_buffer_next];
    ml7396_fifo_read(buf, phr_len + 2 + 1);

    /* --- End ML7396 FIFO critical section --- */
    ml7396_unlock();

    /* set length, ED */
    ml7396_rx_buffer[rx_buffer_next].length = phr_len;
    ml7396_rx_buffer[rx_buffer_next].ed = buf[phr_len + 2 + 1];

    crc = crc_ccitt(0, buf, phr_len);
    crc_received = buf[phr_len] | buf[phr_len + 1] << 8;

    if (crc == crc_received) {
        crc_ok = 1;
    }
    else {
        printf("! [%d] CRC(received) %02x%02x, calc: %04x, status: 0x%08x, len: %d\n",
               page, buf[phr_len+1], buf[phr_len], crc, (unsigned int) status, phr_len);
        crc_ok = 0;
    }

    if (crc_ok == 0) {
        clear_fifo = 1;
        goto ret;
    }

    ml7396_rx_buffer[rx_buffer_next].crc = crc;

    /* read buffer into ieee802154_frame */
    ieee802154_frame_read(buf, &ml7396_rx_buffer[rx_buffer_next].frame,
                          ml7396_rx_buffer[rx_buffer_next].length);

    ieee802154_frame_t *frame;
    uint16_t pan_id;
    uint8_t addr[8];

#if ENABLE_DEBUG
    ieee802154_frame_print_fcf_frame(&ml7396_rx_buffer[rx_buffer_next].frame);
#endif
    frame = &ml7396_rx_buffer[rx_buffer_next].frame;

    pan_id = ml7396_get_pan();
    ml7396_get_address_long_buf(addr);

    /* if packet is no ACK */
    if (ml7396_rx_buffer[rx_buffer_next].frame.fcf.frame_type != IEEE_802154_ACK_FRAME) {
        _send_ack(frame);

        if (ml7396_raw_packet_cb != NULL) {
            fcs_rssi = 0;
            lqi = 0;

            ml7396_raw_packet_cb(&ml7396_netdev, (void*)buf,
                                 ml7396_rx_buffer[rx_buffer_next].length,
                                 fcs_rssi, lqi, (fcs_rssi >> 7));
        }
#if 0
#ifdef MODULE_TRANSCEIVER
        /* notify transceiver thread if any */
        if (transceiver_pid != KERNEL_PID_UNDEF) {
            msg_t m;
            m.type = (uint16_t) RCV_PKT_ML7396;
            m.content.value = rx_buffer_next;
            msg_send_int(&m, transceiver_pid);
        }
#endif
#endif
    }
    else {
        //puts("IEEE_802154_ACK_FRAME");
        if ((pan_id == frame->dest_pan_id) &&
            (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
            memcmp(frame->dest_addr, addr, 8) == 0) {
            if (ml7396_raw_packet_cb != NULL) {
                fcs_rssi = 0;
                lqi = 0;

                ml7396_raw_packet_cb(&ml7396_netdev, (void*)buf,
                                     ml7396_rx_buffer[rx_buffer_next].length,
                                     fcs_rssi, lqi, (fcs_rssi >> 7));
            }
        }
        else {
            puts("to other.");
        }
    }

ret:
    /* shift to next buffer element */
    if (++rx_buffer_next == ML7396_RX_BUF_SIZE) {
        rx_buffer_next = 0;
    }

    /* clear interrupts */
    _ml7396_clear_rx_interrupts(page, clear_fifo);

    last_page = page;
    return page;
}
