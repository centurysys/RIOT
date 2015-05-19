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


static inline void _ml7396_clear_rx_interrupts(int page)
{
    uint32_t interrupts = 0;

    if (page == 0) {
        interrupts |= (INT_RXFIFO0_DONE | INT_RXFIFO0_CRCERR);
    }
    else {
        interrupts |= (INT_RXFIFO1_DONE | INT_RXFIFO1_CRCERR);
    }

    //dprintf("*** clear with: 0x%08x\n", (unsigned int) interrupts);
    ml7396_clear_interrupts(interrupts);
    ml7396_reg_write(ML7396_REG_INT_PD_DATA_IND, 0x00);
}

void ml7396_rx_handler(uint32_t status)
{
    int i, phr_len, page, crc_ok;
    uint8_t *buf, lqi, fcs_rssi;
    uint16_t tmp, crc, crc_received;

    /* detect FIFO page */
    if (status & (INT_RXFIFO0_DONE | INT_RXFIFO0_CRCERR)) {
        page = 0;
    }
    else if (status & (INT_RXFIFO1_DONE | INT_RXFIFO1_CRCERR)) {
        page = 1;
    }
    else {
        return;
    }

    /* read PHR (2-octets) */
    ml7396_fifo_read((uint8_t *) &tmp, 2);

    phr_len = (int) (NTOHS(tmp) & 0x07ff) - 2;
    ml7396_rx_buffer[rx_buffer_next].length = phr_len;

    /* read psdu */
    buf = buffer[rx_buffer_next];
    ml7396_fifo_read(buf, phr_len + 2 + 1);

    /* read ED */
    ml7396_rx_buffer[rx_buffer_next].ed = buf[phr_len + 2 + 1];

#if 0
    for (i = 0; i < phr_len + 2; i++) {
        printf(" %02x", buf[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
    printf("ED: %02x\n", buf[phr_len + 2 + 1]);
#endif

    crc = crc_ccitt(0, buf, phr_len);
    crc_received = buf[phr_len] | buf[phr_len + 1] << 8;

    if (crc == crc_received) {
        crc_ok = 1;
    }
    else {
        printf("! CRC(received) %02x %02x, calc: %04x\n",
               buf[phr_len], buf[phr_len+1], crc);
        crc_ok = 0;
    }

    if (crc_ok == 0) {
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
#if 0
        printf("pan_id: %04x (received: %04x)", pan_id, frame->dest_pan_id);
        if (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) {
            printf(", dest_addr: ");
            for (i = 0; i < 8; i++) {
                printf("%02x", addr[7 - i]);
                if (i < 7)
                    printf("-");
            }
            printf(" (received: ");
            for (i = 0; i < 8; i++) {
                printf("%02x", frame->dest_addr[7 - i]);
                if (i < 7)
                    printf("-");
            }
            puts(")");
        }
        else {
            puts("");
        }

        if ((frame->fcf.ack_req == 1) &&
            (pan_id == frame->dest_pan_id) &&
            (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
            memcmp(frame->dest_addr, addr, 8) == 0) {
            ml7396_send_ack(frame, /* enhanced = */ 1);
        }
#endif

        if (ml7396_raw_packet_cb != NULL) {
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
//        puts("IEEE_802154_ACK_FRAME");
        //ieee802154_frame_print_fcf_frame(&ml7396_rx_buffer[rx_buffer_next].frame);
#if 0
        for (i = 0; i < phr_len; i++) {
            printf(" %02x", buf[i]);
            if ((i % 16) == 15)
                puts("");
        }
        puts("");
#endif
        if ((pan_id == frame->dest_pan_id) &&
            (frame->fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
            memcmp(frame->dest_addr, addr, 8) == 0) {
//            puts("to me.");
            if (ml7396_raw_packet_cb != NULL) {
                ml7396_raw_packet_cb(&ml7396_netdev, (void*)buf,
                                     ml7396_rx_buffer[rx_buffer_next].length,
                                     fcs_rssi, lqi, (fcs_rssi >> 7));
            }
        }
        else {
            puts("to other.");
        }
    }

    /* shift to next buffer element */
    if (++rx_buffer_next == ML7396_RX_BUF_SIZE) {
        rx_buffer_next = 0;
    }

ret:
    /* clear interrupts */
    _ml7396_clear_rx_interrupts(page);
}
