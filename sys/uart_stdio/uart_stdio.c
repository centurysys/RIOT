/*
 * Copyright (C) 2013 INRIA
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys
 * @{
 *
 * @file
 * @brief UART stdio implementation
 *
 * This file implements a UART callback and read/write functions.
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdio.h>

#include "cpu_conf.h"
#include "ringbuffer.h"
#include "thread.h"
#include "mutex.h"
#include "irq.h"

#include "periph/uart.h"

#include "board.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#ifndef STDIO_RX_BUFSIZE
#define STDIO_RX_BUFSIZE (64)
#endif

/**
 * @brief use mutex for waiting on incoming UART chars
 */
static mutex_t _rx_mutex = MUTEX_INIT;
static char _rx_buf_mem[STDIO_RX_BUFSIZE];
static ringbuffer_t _rx_buf;

#ifndef STDIO_TX_BUFSIZE
#define STDIO_TX_BUFSIZE 4096
#endif
static char _tx_buf_mem[STDIO_TX_BUFSIZE];
static ringbuffer_t _tx_buf;

/**
 * @brief Receive a new character from the UART and put it into the receive buffer
 */
void uart_stdio_rx_cb(void *arg, char data)
{
    (void)arg;
    ringbuffer_add_one(&_rx_buf, data);
    mutex_unlock(&_rx_mutex);
}

/**
 * @brief Transmit a new character to the UART
 */
static int uart_stdio_tx_cb(void *arg)
{
    int ret;
    char ch;
    unsigned int intstat;

    intstat = disableIRQ();

    ret = ringbuffer_get_one(&_tx_buf);
    if (ret == -1) {
        restoreIRQ(intstat);
        return 0;
    }

    ch = (char) (ret & 0xff);
    uart_write(STDIO, ch);

    restoreIRQ(intstat);
    return 1;
}

void uart_stdio_init(void)
{
    mutex_lock(&_rx_mutex);
    ringbuffer_init(&_rx_buf, _rx_buf_mem, STDIO_RX_BUFSIZE);
    ringbuffer_init(&_tx_buf, _tx_buf_mem, STDIO_TX_BUFSIZE);
    uart_init(STDIO, STDIO_BAUDRATE, uart_stdio_rx_cb, uart_stdio_tx_cb, 0);
}

int uart_stdio_read(char* buffer, int count)
{
    int res;
    mutex_lock(&_rx_mutex);
    unsigned state = disableIRQ();
    count = (count < _rx_buf.avail) ? count : _rx_buf.avail;
    res = ringbuffer_get(&_rx_buf, (char*)buffer, count);
    restoreIRQ(state);
    return res;
}

int uart_stdio_write(const char* buffer, int len)
{
#if 0
    unsigned int i = len;

    while (i--) {
        uart_write_blocking(STDIO, *buffer++);
    }
#else
    const char *c = buffer;
    unsigned int intstat;

    intstat = disableIRQ();

    for (int i = 0; i < len; i++) {
        if (c[i] == '\n')
            ringbuffer_add_one(&_tx_buf, '\r');
        ringbuffer_add_one(&_tx_buf, c[i]);
    }

    uart_tx_begin(STDIO);

    restoreIRQ(intstat);
#endif
    return len;
}
