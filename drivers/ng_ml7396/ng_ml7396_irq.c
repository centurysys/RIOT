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
 * @brief       Implementation of IRQ thread for ML7396 drivers
 *
 *
 * @}
 */

#include "vtimer.h"

#include "net/ng_netapi.h"
#include "net/ng_pktbuf.h"

#include "ng_ml7396_registers.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_netdev.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define NG_ML7396_IRQ_STACKSIZE (THREAD_STACKSIZE_DEFAULT)
#define NG_ML7396_IRQ_PRIO      (THREAD_PRIORITY_MAIN - 4)

static char _ml7396_irq_stack[NG_ML7396_IRQ_STACKSIZE];

static void *_ng_ml7396_irq_thread(void *args)
{
    ng_netdev_t *dev = (ng_netdev_t *) args;
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                dev->driver->isr_event(dev, msg.content.value);
                break;

            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

kernel_pid_t ng_ml7396_irq_init(ng_ml7396_t *dev)
{
    kernel_pid_t pid;

    if (!dev) {
        return -ENODEV;
    }

    if (dev->irq_pid == KERNEL_PID_UNDEF) {
        pid = thread_create(_ml7396_irq_stack,
                            NG_ML7396_IRQ_STACKSIZE, NG_ML7396_IRQ_PRIO,
                            CREATE_STACKTEST | CREATE_SLEEPING,
                            _ng_ml7396_irq_thread,
                            (void *) dev,
                            "ng_ml7396_IRQ");
        dev->irq_pid = pid;
        printf("%s: irq_pid = %d\n", __FUNCTION__, dev->irq_pid);
        thread_wakeup(pid);
    }

    return dev->irq_pid;
}
