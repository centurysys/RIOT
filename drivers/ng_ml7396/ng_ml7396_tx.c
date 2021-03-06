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
 * @brief       Implementation of TX thread for ML7396 drivers
 *
 *
 * @}
 */

#include "vtimer.h"

#include "net/ng_netapi.h"
#include "net/ng_pktbuf.h"

#include "net/ng_ieee802154.h"
#include "net/ng_netbase.h"

#include "ng_ml7396_registers.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_netdev.h"

#include "ieee802154_frame.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define NG_ML7396_TX_STACKSIZE (THREAD_STACKSIZE_DEFAULT)
#define NG_ML7396_TX_PRIO      (THREAD_PRIORITY_MAIN - 3)

//static kernel_pid_t _ml7396_tx_thread_pid = KERNEL_PID_UNDEF;
static char _ml7396_tx_stack[NG_ML7396_TX_STACKSIZE];


static int _ng_ml7396_wait_ack(ng_ml7396_t *dev, uint8_t seq_nr_expected)
{
    timex_t timeout, t_start, t_end;
    int res;
    msg_t msg;

    timeout.seconds = 0;
    timeout.microseconds = 100 * 1000;

    while (1) {
        vtimer_now(&t_start);
        res = vtimer_msg_receive_timeout(&msg, timeout);
        vtimer_now(&t_end);

        printf("start: %u.%06u, end: %u.%06u\n",
               t_start.seconds, t_start.microseconds,
               t_end.seconds, t_end.microseconds);

        if (res == 1) {
            uint8_t seq_nr = (uint8_t) (msg.content.value & 0xff);

            if (msg.type == MSG_ACK_RECEIVED) {
                if (seq_nr == seq_nr_expected) {
                    printf("*** %s: ACK OK\n", __FUNCTION__);
                    return 0;
                }
                else {
                    printf("%s: seq_nr mismatch, (send: 0x%02x, recv: 0x%02x)\n",
                           __FUNCTION__, seq_nr_expected, seq_nr);
                }
            }
        }
        else {
            puts("ACK timeouted.");
            /* ACK timeouted. */
            break;
        }
    }

    return -1;
}

extern void ps(void);

static void _ng_ml7396_send(ng_ml7396_t *dev, msg_t *msg)
{
    ng_pktsnip_t *pkt;
    ng_netif_hdr_t *hdr;
    size_t sent;
    int i, retry, req_ack;
    u8 seq_nr;

    sent = -ETIMEDOUT;

    pkt = (ng_pktsnip_t *) msg->content.ptr;
    hdr = (ng_netif_hdr_t *) pkt->data;

    if (!(hdr->flags & NG_NETIF_HDR_FLAGS_BROADCAST) &&
        !(hdr->flags & NG_NETIF_HDR_FLAGS_MULTICAST)) {
        req_ack = 1;
        retry = dev->max_retries;
    }
    else {
        req_ack = 0;
        retry = 1;
    }

    for (i = 0; i < retry; i++) {
        seq_nr = dev->seq_nr;

        puts("1");
        sent = ng_ml7396_send_pkt(dev, pkt);
        puts("2");

        if (sent > 0) {
            if (req_ack == 1) {
                puts("3");
                if (_ng_ml7396_wait_ack(dev, seq_nr) == 0) {
                    puts("4");
                    break;
                }
                else {
                    puts("5");
                    sent = -ETIMEDOUT;
                }
            }
            else {
                puts("11");
            }
        }
    }

    puts("6");
    ng_pktbuf_release(pkt);

    msg->content.value = (uint32_t) sent;

    msg_reply(msg, msg);
}

static void *_ng_ml7396_tx_thread(void *args)
{
    ng_ml7396_t *dev = (ng_ml7396_t *) args;
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case NG_NETAPI_MSG_TYPE_SND:
                printf("%s: NG_NETAPI_MSG_TYPE_SND received.\n", __FUNCTION__);
                _ng_ml7396_send(dev, &msg);
                break;

            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

kernel_pid_t ng_ml7396_tx_init(ng_ml7396_t *dev)
{
    kernel_pid_t pid;

    if (!dev) {
        return -ENODEV;
    }

    if (dev->tx_pid == KERNEL_PID_UNDEF) {
        pid = thread_create(_ml7396_tx_stack,
                            NG_ML7396_TX_STACKSIZE, NG_ML7396_TX_PRIO,
                            CREATE_STACKTEST | CREATE_SLEEPING,
                            _ng_ml7396_tx_thread,
                            (void *) dev,
                            "ng_ml7396_tx");
        dev->tx_pid = pid;
        printf("%s: tx_pid = %d\n", __FUNCTION__, dev->tx_pid);
        thread_wakeup(pid);
    }

    return dev->tx_pid;
}
