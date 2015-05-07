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

#include "thread.h"
#include "msg.h"

#include "periph/uart.h"

#include "ml7396.h"
#include "ml7396_spi.h"


kernel_pid_t uart1_handler_pid = KERNEL_PID_UNDEF;

static char uart1_thread_stack[1024];

static void *uart1_thread(void *arg);

void uart1_thread_init(void)
{
    kernel_pid_t pid;

    pid = thread_create(uart1_thread_stack,
                        sizeof(uart1_thread_stack),
                        PRIORITY_MAIN - 1,
                        CREATE_STACKTEST | CREATE_SLEEPING,
                        uart1_thread,
                        NULL,
                        "uart1");
    uart1_handler_pid = pid;
    thread_wakeup(pid);
    puts("uart1_thread_init()");
}

static void _rx_cb(void *arg, char data)
{


}

static int _tx_cb(void *arg)
{


    return 1;
}

static void *uart1_thread(void *arg)
{
    int res;
    msg_t msg;

    uart_init(HOSTIF, HOSTIF_BAUDRATE, _rx_cb, _tx_cb, NULL);

    while (1) {
        timex_t timeout;

        timeout.seconds = 1;
        timeout.microseconds = 0;

        res = vtimer_msg_receive_timeout(&msg, &timeout);
    }

    return NULL;
}
