/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup auto_init_ng_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for nx_ml7396 network interface
 *
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifdef MODULE_NG_ML7396

#include "board.h"
#include "net/ng_nomac.h"
#include "net/ng_netbase.h"

#include "ng_ml7396.h"
#include "ng_ml7396_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define ML7396_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#define ML7396_MAC_PRIO          (THREAD_PRIORITY_MAIN - 3)

#define ML7396_NUM (sizeof(ml7396_params)/sizeof(ml7396_params[0]))

static ng_ml7396_t ng_ml7396_devs[ML7396_NUM];
static char _nomac_stacks[ML7396_MAC_STACKSIZE][ML7396_NUM];

void auto_init_ng_ml7396(void)
{
    int i, res;
    const ml7396_params_t *p;

    for (i = 0; i < ML7396_NUM; i++) {
        DEBUG("Initializing Ml7396 radio at SPI_%i\n", i);
        p = &ml7396_params[i];

        res = ng_ml7396_init(&ng_ml7396_devs[i],
                             p->spi,
                             p->spi_speed,
                             p->cs_pin,
                             p->int_pin,
                             p->reset_pin,
                             p->tcxo_pin);

        if (res < 0) {
            DEBUG("Error initializing Ml7396 radio device!");
        }
        else {
            ng_nomac_init(_nomac_stacks[i],
                    ML7396_MAC_STACKSIZE, ML7396_MAC_PRIO,
                    "ml7396", (ng_netdev_t *)&ng_ml7396_devs[i]);
        }
    }
}
#else
typedef int dont_be_pedantic;
#endif /* MODULE_NG_ML7396 */

/** @} */
