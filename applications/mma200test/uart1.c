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

    len = (int) dev->params[0];

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

    mutex_lock(&(uart_dev.cmd_mutex));

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
    int i;
    timex_t now;

    //printf("* %s (len: %d)\n", __FUNCTION__, buflen);

    mutex_lock(&dev->tx_mutex);

    dev->send_buf[0] = 's';
    dev->send_buf[1] = '2';
    dev->send_buf[2] = code;
    dev->send_buf[3] = (uint8_t) (buflen & 0xff);
    memcpy(&dev->send_buf[4], buf, buflen);

    dev->send_len = buflen + 4;
    dev->send_idx = 0;

#if 0
    printf("%s: received_data\n", __FUNCTION__);
    for (i = 0; i < buflen; i++) {
        printf(" %02x", buf[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

    vtimer_now(&now);
    printf("%s[%lu.%06lu]:\n", __FUNCTION__, now.seconds, now.microseconds);

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
 *
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
 *
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
 *
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
 *
 */
static void _cmd_transmit_blk(uart1_dev_t *dev)
{
    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;

    /* send to Radio */
    msg.type = MSG_UART_RECEIVED;
    msg_send_int(&msg, uart1_rx_handler_pid);

    //printf("%s: wait for sent...\n", __FUNCTION__);
    //mutex_lock(&dev->cmd_mutex);
    //printf("%s: ---> sent.\n", __FUNCTION__);

#if 0
    /* response to host */
    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_TRANSMIT_BLK);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);
#endif
}

typedef void (*cmd_handler_t)(uart1_dev_t *dev);

struct cmd_handler {
    char cmd;
    cmd_handler_t func;
};

static struct cmd_handler cmd_handlers[] = {
    { CMD_NOOP, _cmd_noop },
    { CMD_OPEN, _cmd_open },
    { CMD_CLOSE, _cmd_close },
    { CMD_TRANSMIT_BLK, _cmd_transmit_blk },
    { -1, NULL }
};

/*
 *
 */
static void _process_cmd(uart1_dev_t *dev)
{
    struct cmd_handler *handler;

    //printf("%s ...\n", __FUNCTION__);

    for (handler = &cmd_handlers[0]; handler->func; handler++) {
        if (dev->cmd == handler->cmd) {
            handler->func(dev);
            break;
        }
    }
}

static void handle_uart1_recv(uart1_dev_t *dev, char data);

/*
 *
 */
static void _rx_cb(void *arg, char data)
{
    uart1_dev_t *dev = (uart1_dev_t *) arg;

    handle_uart1_recv(dev, data);
}

/*
 *
 */
static void handle_uart1_recv(uart1_dev_t *dev, char data)
{
    int next_state = STATE_UNDEF;

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

    _get_msgbuf(&msgbuf, &_buf);

#if 1
//    _buf[0] = lqi;
//    _buf[1] = (uint8_t) (len & 0xff);
    memcpy(_buf, buf, len);
#endif

    msgbuf->code = CMD_RECEIVE_BLK;
    msgbuf->buf  = _buf;
    msgbuf->len  = len;

    msg.type = MSG_RF_RECEIVED;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);

    printf("%s: called, len = %d\n", __FUNCTION__, len);
}


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

static char transmit_buf[256];
static int transmit_len;

static int _transmit(uart1_dev_t *dev)
{
    int i;
    int res;
    timex_t start, end, diff;

    //printf("%s: data_len = %d\n", __FUNCTION__, dev->data_len);

    vtimer_now(&start);

    memcpy(transmit_buf, dev->data, dev->data_len);
    transmit_len = dev->data_len;

    //res = ml7396_send_raw(dev->data, dev->data_len);
    res = ml7396_send_raw(transmit_buf, transmit_len);

    vtimer_now(&end);

#if 0
    for (i = 0; i < transmit_len; i++) {
        printf(" %02x", transmit_buf[i]);
        if ((i % 16) == 15)
            puts("");
    }
    puts("");
#endif

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
    printf("%s[%lu.%06lu]: ---> sent, %lu.%06lu\n", __FUNCTION__,
           end.seconds, end.microseconds, diff.seconds, diff.microseconds);

    msg_t msg;
    msgbuf_t *msgbuf;
    char *buf;

    /* response to host */
    _get_msgbuf(&msgbuf, &buf);

    buf[0] = STATUS_SUCCESS;

    msgbuf->code = RESPONSE(CMD_TRANSMIT_BLK);
    msgbuf->buf  = buf;
    msgbuf->len  = 1;

    msg.type = MSG_SEND_REQ;
    msg.content.ptr = (char *) msgbuf;

    msg_send(&msg, uart1_tx_handler_pid);

    return res;
}


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
                    break;
            }
        }
    }

    return NULL;
}
