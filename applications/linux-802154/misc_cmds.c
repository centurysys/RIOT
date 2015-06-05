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
#include "cpu.h"
#include "irq.h"

#include "periph/random.h"

#include "utils.h"

/*
 *
 */
int cmd_vtimer_test(int argc, char **argv)
{
    timex_t now;

    vtimer_now(&now);

    printf("seconds = %lu, microseconds = %lu\n", now.seconds, now.microseconds);
    return 0;
}

/*
 *
 */
int cmd_random_test(int argc, char **argv)
{
    unsigned int val;

    if (random_read((char *) &val, 4) == 4) {
        printf("random: 0x%08x\n", val);
    }

    return 0;
}

int cmd_conf_pins(int argc, char **argv)
{
    uint8_t conf;

    conf = get_config_pins();

    printf("CONF: 0x%02x\n", (unsigned int) conf);

    return 0;
}

typedef void (*pfunc)(void);

static pfunc application;
static uint32_t jump_address;

#define APPLICATION_ADDRESS 0x08060000

int cmd_jump_to_app(int argc, char **argv)
{
    unsigned int state;

    /* check "application"'s stack */
    if (((*((uint32_t *) APPLICATION_ADDRESS)) & 0x2ffe0000) == 0x20000000) {
        jump_address = *((uint32_t *) (APPLICATION_ADDRESS + 4));
        application = (pfunc) jump_address;

        state = disableIRQ();
        __set_MSP(*((uint32_t *) APPLICATION_ADDRESS));

        application();

        restoreIRQ(state);
    }
    else {
        puts("app: application not programmed.");
    }

    return -1;
}
