/*
 * Copyright (C) 2015
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vtimer.h>

#include "thread.h"
#include "msg.h"

#include "periph/uart.h"

#include "ml7396.h"
#include "ml7396_spi.h"

#include "uart1.h"

enum uart1_rx_state {
    STATE_UNDEF,

    STATE_IDLE,
    STATE_WAIT_START2,
    STATE_WAIT_COMMAND,
    STATE_WAIT_PARAMS,
    STATE_WAIT_DATA,
};

#define START_CODE1  's'
#define START_CODE2  '2'

enum cmd_res_code {
    CMD_NOOP           = 0x00,
    CMD_OPEN           = 0x01,
    CMD_CLOSE          = 0x02,
    CMD_SET_CHANNEL    = 0x03,
    CMD_TRANSMIT_BLK   = 0x04,
    CMD_RECEIVE_BLK    = 0x05,
    CMD_GET_LONG_ADDR  = 0x06,
    CMD_ENERGY_DET     = 0x07,
    CMD_SET_LONG_ADDR  = 0x08,
    CMD_SET_SHORT_ADDR = 0x09,
    CMD_SET_PAN_ID     = 0x0a,
    CMD_SET_PROMISC    = 0x0b,
};

#define RESPONSE(x)  ((x) + 0x80)

enum status_code {
    STATUS_SUCCESS            = 0x00,
    STATUS_FAILURE            = 0x01,
    STATUS_SUCCESS_WITH_EXTRA = 0x02,
};

enum error_code {
    ERR_NONE             = 0x00,
    ERR_BUSY_RX          = 0x01,
    ERR_BUSY_TX          = 0x02,
    ERR_BUSY_UNSPEC      = 0x03,
    ERR_TRX_OFF          = 0x04,
    ERR_UNSUPPORTED_CHAN = 0x05,
    ERR_UNSUPPORTED_PAGE = 0x06,
    ERR_NOT_IMPLEMENTED  = 0x07,
    ERR_UNKNOWN          = 0x08,
};


typedef struct uart1_dev {
    int opened;
    int state;

    char cmd;
    char params[16];
    char data[255];

    int param_len;
    int data_len;

    int param_idx;
    int data_idx;

    mutex_t cmd_mutex;

    /* TX side */
    mutex_t tx_mutex;

    char send_buf[256];
    int send_len;
    int send_idx;
} uart1_dev_t;

static void rf_receive_raw_cb(netdev_t *dev, void *buf, size_t len,
                              int8_t rssi, uint8_t lqi, int crc_ok);

static void _cleanup(uart1_dev_t *dev);


/*
 *
 */
static int get_param_len(char cmd)
{
    int len;

    switch (cmd) {
        case CMD_NOOP:
        case CMD_OPEN:
        case CMD_CLOSE:
        case CMD_GET_LONG_ADDR:
        case CMD_ENERGY_DET:
            len = 0;
            break;

        case CMD_SET_PROMISC:
        case CMD_TRANSMIT_BLK:
            len = 1;
            break;

        case CMD_SET_CHANNEL:
        case CMD_SET_SHORT_ADDR:
        case CMD_SET_PAN_ID:
            len = 2;
            break;

        case CMD_SET_LONG_ADDR:
            len = 8;
            break;

        default:
            len = -1;
    }

    return len;
}

/*
 *
 */
static int get_data_len(uart1_dev_t *dev)
{
    int len;

    if (dev->cmd == CMD_TRANSMIT_BLK) {
        len = (int) dev->params[0];
    }
    else {
        len = -1;
    }

    //printf("datalen = %d\n", len);
    return len;
}


static uart1_dev_t uart_dev;

kernel_pid_t uart1_rx_handler_pid = KERNEL_PID_UNDEF;
kernel_pid_t uart1_tx_handler_pid = KERNEL_PID_UNDEF;

static char uart1_tx_thread_stack[1024];
static char uart1_rx_thread_stack[1024];

static void *uart1_rx_thread(void *arg);
static void *uart1_tx_thread(void *arg);


/*
 *
 */
void uart1_thread_init(void)
{
    kernel_pid_t pid;
    const netdev_802154_driver_t *driver;

    _cleanup(&uart_dev);
    mutex_init(&(uart_dev.tx_mutex));
    mutex_init(&(uart_dev.cmd_mutex));

    //mutex_lock(&(uart_dev.cmd_mutex));

    /* create UART1 RX thread */
    pid = thread_create(uart1_rx_thread_stack,
                        sizeof(uart1_rx_thread_stack),
                        //PRIORITY_MAIN - 1,
                        3,
                        CREATE_STACKTEST | CREATE_SLEEPING,
                        uart1_rx_thread,
                        &uart_dev,
                        "uart1_rx");
    uart1_rx_handler_pid = pid;
    thread_wakeup(pid);
    printf("uart1_thread_init[RX] -> pid is %d\n", (int) pid);

    /* create UART1 TX thread */
    pid = thread_create(uart1_tx_thread_stack,
                        sizeof(uart1_tx_thread_stack),
                        //PRIORITY_MAIN - 1,
                        3,
                        CREATE_STACKTEST | CREATE_SLEEPING,
                        uart1_tx_thread,
                        &uart_dev,
                        "uart1_tx");
    uart1_tx_handler_pid = pid;
    thread_wakeup(pid);
    printf("uart1_thread_init[TX] -> pid is %d\n", (int) pid);

    driver = (const netdev_802154_driver_t *) ml7396_netdev.driver;
    driver->add_receive_raw_callback(&ml7396_netdev, rf_receive_raw_cb);
}

/*
 *
 */
static void send_uart(uart1_dev_t *dev, char code, char *buf, int buflen)
{
    //printf("* %s (len: %d)\n", __FUNCTION__, buflen);

    mutex_lock(&dev->tx_mutex);

    dev->send_buf[0] = 's';
    dev->send_buf[1] = '2';
    dev->send_buf[2] = code;
    memcpy(&dev->send_buf[3], buf, buflen);

    dev->send_len = buflen + 3;
    dev->send_idx = 0;

    uart_tx_begin(HOSTIF);
    //printf("tx_begin\n");
}

static void send_uart2(uart1_dev_t *dev, char code, char *buf, int buflen)
{
    timex_t now;

    mutex_lock(&dev->tx_mutex);

    dev->send_buf[0] = 's';
    dev->send_buf[1] = '2';
    dev->send_buf[2] = code;
    //dev->send_buf[3] = 0xcc;
    //dev->send_buf[4] = (uint8_t) (buflen & 0xff);
    memcpy(&dev->send_buf[3], buf, buflen);

    dev->send_len = buflen + 3;
    dev->send_idx = 0;

#if 0
    printf("%s: received_data\n", __FUNCTION__);
    int i;

    for (i = 0; i < buflen; i++) {
        printf(" %02x", buf[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

    vtimer_now(&now);
    //printf("%s[%lu.%06lu]:\n", __FUNCTION__, now.seconds, now.microseconds);

    uart_tx_begin(HOSTIF);
    //printf("tx_begin\n");
}

/*
 *
 */
static void _cleanup(uart1_dev_t *dev)
{
    dev->state = STATE_IDLE;

    dev->param_len = -1;
    dev->data_len = -1;

    dev->param_idx = 0;
    dev->data_idx = 0;
}


/*
 *  UART commands
 */

#define MSGBUF_NUMS 3
static msgbuf_t _msgbuf[MSGBUF_NUMS];
static char _response_buf[MSGBUF_NUMS][256];
static volatile int msgbuf_next;

static void _get_msgbuf(msgbuf_t **msgbuf, char **buf)
{
    *msgbuf = &_msgbuf[msgbuf_next];
    *buf = _response_buf[msgbuf_next];

    //printf("%s: next = %d\n", __FUNCTION__, msgbuf_next);
    //printf(" msgbuf: %p, buf: %p\n", *msgbuf, *buf);

    if (++msgbuf_next >= MSGBUF_NUMS)
        msgbuf_next = 0;
}

/*
 *  command handler: CMD_NOOP (0x00)
 */
static void _cmd_noop(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;

    _get_msgbuf(&msgbuf, &buf);

    dev->opened = 1;

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_NOOP);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_OPEN (0x01)
 */
static void _cmd_open(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;

    _get_msgbuf(&msgbuf, &buf);

    dev->opened = 1;

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_OPEN);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_CLOSE (0x02)
 */
static void _cmd_close(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;

    _get_msgbuf(&msgbuf, &buf);

    dev->opened = 0;

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_CLOSE);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_SET_CHANNEL (0x03)
 */
static void _cmd_set_channel(uart1_dev_t *dev)
{
    uint8_t page, ch_no;
    unsigned int channel;
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf, err, status;

    page = (uint8_t) dev->params[0];
    ch_no = (uint8_t) dev->params[1];

    status = STATUS_FAILURE;
    err = ERR_NONE;

    if (page != 9) {
        err = ERR_UNSUPPORTED_PAGE;
    }
    else if (ch_no < 4 || ch_no > 17) {
        err = ERR_UNSUPPORTED_CHAN;
    }
    else {
        int res;

        status = STATUS_SUCCESS;

        channel = 33 + (ch_no - 4) * 2;
        res = ml7396_set_channel(channel);

        if (res < 0) {
            status = STATUS_FAILURE;
            err = ERR_UNSUPPORTED_CHAN;
        }
    }

    _get_msgbuf(&msgbuf, &buf);

    buf[0] = status;
    buf[1] = err;

    msgbuf->code = RESPONSE(CMD_SET_CHANNEL);
    msgbuf->buf  = buf;
    msgbuf->len  = 2;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_TRANSMIT_BLOCK (0x05)
 */
static void _cmd_transmit_blk(uart1_dev_t *dev)
{
    msg_t msg;

    /* send to Radio */
    msg.type = MSG_UART_RECEIVED;
    msg_send_int(&msg, uart1_rx_handler_pid);

    /* response may be sent later from TX thread. */
}

/*
 *  command handler: CMD_GET_LONG_ADDR (0x06)
 */
static void _cmd_get_long_addr(uart1_dev_t *dev)
{
    int i;
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;
    uint64_t long_addr;

    long_addr = ml7396_get_address_long();

    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;

    for (i = 0; i < 8; i++) {
        buf[1 + i] = (char) ((long_addr >> (8 * (7 - i))) & 0xff);
    }

    msgbuf->code = RESPONSE(CMD_GET_LONG_ADDR);
    msgbuf->buf  = buf;
    msgbuf->len  = 9;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_SET_LONG_ADDR (0x08)
 */
static void _cmd_set_long_addr(uart1_dev_t *dev)
{
    int i;
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;
    uint64_t long_addr;

    long_addr = 0;
    for (i = 0; i < 8; i++) {
        long_addr |= ((uint64_t) dev->params[i]) << (8 * i);
        printf(" %02x", dev->params[i]);
    }
    puts("");

    ml7396_set_address_long(long_addr);

    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_SET_LONG_ADDR);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    printf("%s: success\n", __FUNCTION__);
    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_SET_SHORT_ADDR (0x09)
 */
static void _cmd_set_short_addr(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;
    uint16_t short_addr;

    short_addr = ((((uint16_t) dev->params[0]) << 8) |
                  ((uint16_t) dev->params[1]));

    ml7396_set_address(short_addr);

    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;
    buf[1] = ERR_NONE;

    msgbuf->code = RESPONSE(CMD_SET_SHORT_ADDR);
    msgbuf->buf  = buf;
    msgbuf->len  = 2;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 *  command handler: CMD_SET_PAN_ID (0x0a)
 */
static void _cmd_set_pan_id(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;
    uint16_t pan_id;

    pan_id = ((((uint16_t) dev->params[0]) << 8) |
              ((uint16_t) dev->params[1]));

    ml7396_set_pan(pan_id);

    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;
    buf[1] = ERR_NONE;

    msgbuf->code = RESPONSE(CMD_SET_PAN_ID);
    msgbuf->buf  = buf;
    msgbuf->len  = 2;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
}

/*
 * serial protocol v2 command handlers
 */
typedef void (*cmd_handler_t)(uart1_dev_t *dev);

struct cmd_handler {
    char cmd;
    cmd_handler_t func;
};

static struct cmd_handler cmd_handlers[] = {
    { CMD_NOOP, _cmd_noop },
    { CMD_OPEN, _cmd_open },
    { CMD_CLOSE, _cmd_close },
    { CMD_SET_CHANNEL, _cmd_set_channel },
    { CMD_TRANSMIT_BLK, _cmd_transmit_blk },
    { CMD_GET_LONG_ADDR, _cmd_get_long_addr },
    { CMD_SET_LONG_ADDR, _cmd_set_long_addr },
    { CMD_SET_SHORT_ADDR, _cmd_set_short_addr },
    { CMD_SET_PAN_ID, _cmd_set_pan_id },
//  { CMD_SET_PROMISC, _cmd_set_promisc },
    { -1, NULL }
};

/*
 *
 */
static void _process_cmd(uart1_dev_t *dev)
{
    struct cmd_handler *handler;

    //printf("%s: cmd: %02x\n", __FUNCTION__, dev->cmd);

    for (handler = &cmd_handlers[0]; handler->func; handler++) {
        if (dev->cmd == handler->cmd) {
            handler->func(dev);
            break;
        }
    }
}

static void handle_uart1_recv(uart1_dev_t *dev, char data);

/*
 *  UART driver callback (transmit)
 */
static int _tx_cb(void *arg)
{
    uart1_dev_t *dev = (uart1_dev_t *) arg;

    if (dev->send_idx >= dev->send_len) {
        mutex_unlock(&dev->tx_mutex);

        return 0;
    }

    uart_write(HOSTIF, dev->send_buf[dev->send_idx++]);

    return 1;
}

/*
 *  UART driver callback (receive)
 */
static void _rx_cb(void *arg, char data)
{
    uart1_dev_t *dev = (uart1_dev_t *) arg;

    handle_uart1_recv(dev, data);
}

static void timex_diff(timex_t *start, timex_t *end, timex_t *diff)
{
    if (start->seconds == 0 && start->microseconds == 0) {
        diff->seconds = 0;
        diff->microseconds = 0;
    }
    else {
        diff->seconds = end->seconds - start->seconds;
        if (start->microseconds > end->microseconds) {
            diff->microseconds = 1 * 1000 * 1000 + end->microseconds - start->microseconds;
            diff->seconds -= 1;
        }
        else {
            diff->microseconds = end->microseconds - start->microseconds;
        }
    }
}


/*
 *
 */
static void handle_uart1_recv(uart1_dev_t *dev, char data)
{
    int next_state = STATE_UNDEF;
    static timex_t last = {0, 0};
    timex_t now, diff;
    uint32_t diff_us;

    vtimer_now(&now);
    timex_diff(&last, &now, &diff);
    last = now;

    diff_us = diff.seconds * (1 * 1000 * 1000) + diff.microseconds;

    if (dev->state != STATE_IDLE && diff_us > (50 * 1000)) {
        puts("UART timeouted.");
        _cleanup(dev);
    }

    switch (dev->state) {
        case STATE_IDLE:
            if (data == START_CODE1) {
                next_state = STATE_WAIT_START2;
            }
            break;

        case STATE_WAIT_START2:
            if (data == START_CODE2) {
                next_state = STATE_WAIT_COMMAND;
            }
            break;

        case STATE_WAIT_COMMAND:
            dev->cmd = data;
            dev->param_len = get_param_len(data);
            dev->param_idx = 0;

            if (dev->param_len == 0) {
                _process_cmd(dev);
            }
            else if (dev->param_len > 0) {
                next_state = STATE_WAIT_PARAMS;
            }
            break;

        case STATE_WAIT_PARAMS:
            dev->params[dev->param_idx++] = data;

            if (dev->param_idx == dev->param_len) {
                dev->data_len = get_data_len(dev);
                dev->data_idx = 0;

                if (dev->data_len == -1) {
                    /* データ無し */
                    _process_cmd(dev);
                    next_state = STATE_IDLE;
                }
                else if (dev->data_len > 0) {
                    next_state = STATE_WAIT_DATA;
                }
            }
            else {
                next_state = STATE_WAIT_PARAMS;
            }
            break;

        case STATE_WAIT_DATA:
            dev->data[dev->data_idx++] = data;

            if (dev->data_idx == dev->data_len) {
                _process_cmd(dev);
                next_state = STATE_IDLE;
            }
            else {
                next_state = STATE_WAIT_DATA;
            }
            break;

        default:
            break;
    }

    if (next_state == STATE_UNDEF) {
        _cleanup(dev);
    }
    else {
        dev->state = next_state;
    }
}

static void rf_receive_raw_cb(netdev_t *dev, void *buf, size_t len,
                              int8_t rssi, uint8_t lqi, int crc_ok)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *_buf;
    ieee802154_frame_t frame;

    ieee802154_frame_read(buf, &frame, len);

    if (frame.fcf.frame_type != IEEE_802154_ACK_FRAME) {
        _get_msgbuf(&msgbuf, &_buf);

#if 1
        _buf[0] = lqi;
        _buf[1] = (uint8_t) (len & 0xff);
        memcpy(&_buf[2], buf, len);
#endif

        msgbuf->code = CMD_RECEIVE_BLK;
        msgbuf->buf  = _buf;
        msgbuf->len  = len + 2;

        msg.type = MSG_RF_RECEIVED;
        msg.content.ptr = (char *) msgbuf;

        msg_send(&msg, uart1_tx_handler_pid);
    }
    else {
        /* ACK received */
        //puts("ACK received.");
        msg.type = MSG_ACK_RECEIVED;
        msg_send(&msg, uart1_rx_handler_pid);
    }
}


static void send_ack(uart1_dev_t *dev, char *buf, int buflen)
{
    ieee802154_frame_t frame;
    uint16_t pan_id;
    uint8_t addr[8];

    ieee802154_frame_read(buf, &frame, buflen);

    pan_id = ml7396_get_pan();
    ml7396_get_address_long_buf(addr);

    if ((frame.fcf.ack_req == 1) &&
        (pan_id == frame.dest_pan_id) &&
        (frame.fcf.dest_addr_m == IEEE_802154_LONG_ADDR_M) &&
        memcmp(frame.dest_addr, addr, 8) == 0) {
        ml7396_send_ack(&frame, /* enhanced = */ 1);

        //printf("%s: send ACK\n", __FUNCTION__);
    }
}


static char transmit_buf[256];
static int transmit_len;

static void backoff(void)
{


}

static int _transmit(uart1_dev_t *dev)
{
    int res, wait_ack, retry, i;
    ieee802154_frame_t frame;
    msg_t msg;
    timex_t timeout;
//  timex_t start, end, diff;

    //printf("%s: data_len = %d\n", __FUNCTION__, dev->data_len);

//  vtimer_now(&start);

    memcpy(transmit_buf, dev->data, dev->data_len);
    transmit_len = dev->data_len;

    ieee802154_frame_read((uint8_t *) dev->data, &frame, dev->data_len);

    //printf("%s: ack_req = %d\n", __FUNCTION__, frame.fcf.ack_req);

    if (frame.fcf.ack_req == 1) {
        retry = 3;
        wait_ack = 1;
    }
    else {
        retry = 1;
        wait_ack = 0;
    }

    for (i = 0; i < retry; i++) {
        res = ml7396_send_raw(dev->data, dev->data_len);

        if (res == 0 && wait_ack == 1) {
            timeout.seconds = 0;
            timeout.microseconds = 5 * 1000; /* 5ms */

            res = vtimer_msg_receive_timeout(&msg, timeout);
            //printf("res=%d\n", res);

            if (res == 1) {
                res = 0;
                break;
            }
            else {
                res = -ETIMEDOUT;
                usleep(10 * 1000);
            }
        }
    }

//  vtimer_now(&end);
    if (res != 0) {
        printf("%s: timeouted...\n", __FUNCTION__);
    }

#if 0
    int i;

    for (i = 0; i < transmit_len; i++) {
        printf(" %02x", transmit_buf[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

#if 0
    diff.microseconds = end.microseconds - start.microseconds;

    if (end.microseconds < start.microseconds) {
        diff.microseconds = 1 * 1000 * 1000 + end.microseconds - start.microseconds;
        diff.seconds = end.seconds - start.seconds - 1;
    }
    else {
        diff.microseconds = end.microseconds - start.microseconds;
        diff.seconds = end.seconds - start.seconds;
    }

    //mutex_unlock(&dev->cmd_mutex);
    //printf("%s[%lu.%06lu]: ---> sent, %lu.%06lu\n", __FUNCTION__,
    //end.seconds, end.microseconds, diff.seconds, diff.microseconds);
#endif

    msgbuf_t *msgbuf;
    char *buf;

    /* response to host */
    _get_msgbuf(&msgbuf, &buf);

    buf[0] = (res == 0) ? STATUS_SUCCESS : STATUS_FAILURE;

    msgbuf->code = RESPONSE(CMD_TRANSMIT_BLK);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);

    return res;
}


/*
 * UART1: TX thread
 */
static void *uart1_tx_thread(void *arg)
{
    uart1_dev_t *dev;
    msgbuf_t *msgbuf;
    int res;
    msg_t msg;

    dev = (uart1_dev_t *) arg;

    while (1) {
        res = msg_receive(&msg);

        if (res == 1) {
            switch (msg.type) {
                case MSG_SEND_REQ:
                    msgbuf = (msgbuf_t *) msg.content.ptr;

                    /* maybe block */
                    send_uart(dev, msgbuf->code, msgbuf->buf, msgbuf->len);
                    break;

                case MSG_RF_RECEIVED:
                    msgbuf = (msgbuf_t *) msg.content.ptr;

                    send_ack(dev, &(msgbuf->buf[2]), msgbuf->len - 2);

                    /* maybe block */
                    send_uart2(dev, CMD_RECEIVE_BLK, msgbuf->buf, msgbuf->len);
                    break;

                default:
                    break;
            }
        }
    }

    return NULL;
}


/*
 * UART1: RX thread
 */
static void *uart1_rx_thread(void *arg)
{
    uart1_dev_t *dev;
    int res;
    msg_t msg;

    dev = (uart1_dev_t *) arg;

    uart_init(HOSTIF, HOSTIF_BAUDRATE, _rx_cb, _tx_cb, dev);

    while (1) {
        //printf("%s: wait msg forever...\n", __FUNCTION__);
        res = msg_receive(&msg);

        if (res == 1) {
            //printf("%s: message received\n", __FUNCTION__);

            switch (msg.type) {
                case MSG_UART_RECEIVED:
                    _transmit(dev);
                    break;

                default:
                    printf("%s: msg.type = %d\n", __FUNCTION__, msg.type);
                    break;
            }
        }
    }

    return NULL;
}
