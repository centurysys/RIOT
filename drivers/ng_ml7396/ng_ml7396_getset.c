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
 * @brief       Getter and setter functions for the ML7396 driver
 *
 *
 * @}
 */

#include "ng_ml7396.h"
#include "ng_ml7396_internal.h"
#include "ng_ml7396_registers.h"
#include "periph/spi.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


uint16_t ng_ml7396_get_addr_short(ng_ml7396_t *dev)
{
    return (dev->addr_short[0] << 8) | dev->addr_short[1];
}

void ng_ml7396_set_addr_short(ng_ml7396_t *dev, uint16_t addr)
{
    dev->addr_short[0] = addr >> 8;
    dev->addr_short[1] = addr & 0xff;

    ng_ml7396_reg_write(dev, ML7396_REG_SHT_ADDR0_H,
                        dev->addr_short[0]);
    ng_ml7396_reg_write(dev, ML7396_REG_SHT_ADDR0_L,
                        dev->addr_short[1]);
}

uint64_t ng_ml7396_get_addr_long(ng_ml7396_t *dev)
{
    int i;
    uint64_t addr;
    uint8_t *ap = (uint8_t *) (&addr);

    for (i = 0; i < 8; i++) {
        ap[i] = dev->addr_long[7 - i];
    }

    return addr;
}

void ng_ml7396_set_addr_long(ng_ml7396_t *dev, uint64_t addr)
{
    int i;

    for (i = 0; i < 8; i++) {
        /* addr: big-endian */
        dev->addr_long[i] = (addr >> ((7 - i) * 8));
    }

    ng_ml7396_reg_writes(dev, ML7396_REG_64ADDR1, dev->addr_long, 8);
}

uint8_t ng_ml7396_get_chan(ng_ml7396_t *dev)
{
    return dev->chan;
}

uint16_t ng_ml7396_get_pan(ng_ml7396_t *dev)
{
    return dev->pan;
}

void ng_ml7396_set_pan(ng_ml7396_t *dev, uint16_t pan)
{
    dev->pan = pan;
    DEBUG("pan0: %u, pan1: %u\n", (uint8_t) pan, pan >> 8);

    ng_ml7396_reg_write(dev, ML7396_REG_PANID_L, (uint8_t) pan);
    ng_ml7396_reg_write(dev, ML7396_REG_PANID_H, (pan >> 8));
}

uint8_t ng_ml7396_get_max_retries(ng_ml7396_t *dev)
{
    return dev->max_retries;
}

void ng_ml7396_set_max_retries(ng_ml7396_t *dev, uint8_t max)
{
    dev->max_retries = max;
}

void ng_ml7396_set_option(ng_ml7396_t *dev, uint16_t option, bool state)
{
    DEBUG("set option %i to %i\n", option, state);

    /* set option field */
    if (state) {
        dev->options |= option;
        /* trigger option specific actions */
        switch (option) {
            case NG_ML7396_OPT_CSMA:
                DEBUG("[ng_ml7396] opt: enabling CSMA mode (NOT IMPLEMENTED)\n");
                break;

            case NG_ML7396_OPT_PROMISCUOUS:
                DEBUG("[ng_ml7396] opt: enabling PROMISCUOUS mode\n");
                break;

            default:
                /* do nothing */
                break;
        }
    }
    else {
        dev->options &= ~(option);
        /* trigger option specific actions */
        switch (option) {
            case NG_ML7396_OPT_CSMA:
                DEBUG("[ng_ml7396] opt: disabling CSMA mode (NOT IMPLEMENTED)\n");
                /* TODO: en/disable csma */
                break;

            case NG_ML7396_OPT_PROMISCUOUS:
                DEBUG("[ng_ml7396] opt: disabling PROMISCUOUS mode\n");
                /* disable promiscuous mode */
                break;

            default:
                /* do nothing */
                break;
        }
    }
}

uint8_t ng_ml7396_get_state(ng_ml7396_t *dev)
{
    uint8_t status;

    //status = ng_ml7396_get_status(dev);
    return status;
}

static inline void _set_state(ng_ml7396_t *dev, uint8_t state)
{


}

void ng_ml7396_set_state(ng_ml7396_t *dev, uint8_t state)
{
    uint8_t old_state = ng_ml7396_get_state(dev);

    if (state == old_state) {
        return;
    }



}
