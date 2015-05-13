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

#include "ml7396.h"
#include "ml7396_spi.h"

/*
 *
 */
int cmd_ml7396_reads(int argc, char **argv)
{
    int i, bank = 0;
    uint8_t buf;

    if (argc >= 2) {
        bank = atoi(argv[1]);

        if (bank < 0 || bank > 2)
            bank = 0;
    }

    printf("bank: %d\n", bank);

    for (i = 0; i < 0x7e; i++) {
        buf = ml7396_reg_read(((uint16_t) (bank | 0x80)) << 8 | i << 1);
        printf("[0x%02x] 0x%02x\n", i, buf);
    }

    return 0;
}

int cmd_ml7396_get_interrupt_status(int argc, char **argv)
{
    uint32_t status;

    status = ml7396_get_interrupt_status();
    printf("Interrupt Status: 0x%08x\n", (unsigned int) status);

    status = ml7396_get_interrupt_enable();
    printf("Interrupt Enable: 0x%08x\n", (unsigned int) status);

    return 0;
}

extern void ml7396_reset(void);

int cmd_ml7396_reset(int argc, char **argv)
{
    ml7396_reset();

    return 0;
}

int cmd_ml7396_regwrite(int argc, char **argv)
{
    uint8_t bank, addr, val;
    uint16_t reg;

    if (argc < 3)
        return -1;

    bank = (uint8_t) strtoul(argv[1], NULL, 16);
    addr = (uint8_t) strtoul(argv[2], NULL, 16);
    val  = (uint8_t) strtoul(argv[3], NULL, 16);

    if (bank < 0 || bank > 2)
        return -1;

    printf("bank = 0x%02x, addr = 0x%02x, val = 0x%02x\n",
           bank, addr, val);

    reg = (bank << 8) | (addr << 1) | 1;
    printf("reg: 0x%04x\n", reg);

    ml7396_reg_write(reg, val);
    val = ml7396_reg_read(reg & ~1);

    printf(" --> readback: 0x%02x\n", val);

    return 0;
}

int cmd_ml7396_regread(int argc, char **argv)
{
    uint8_t bank, addr, val;
    uint16_t reg;

    if (argc < 2)
        return -1;

    bank = (uint8_t) strtoul(argv[1], NULL, 16);
    addr = (uint8_t) strtoul(argv[2], NULL, 16);

    if (bank < 0 || bank > 2)
        return -1;

    reg = (bank << 8) | (addr << 1) | 1;
    val = ml7396_reg_read(reg & ~1);

    printf("bank = 0x%02x, addr = 0x%02x -> 0x%02x\n",
           bank, addr, val);

    return 0;
}

int cmd_cca(int argc, char **argv)
{
    printf("%s: call ml7396_channel_is_clear...\n", __FUNCTION__);

    ml7396_channel_is_clear(&ml7396_netdev);

    return 0;
}

int cmd_fifo_write(int argc, char **argv)
{
#if 1
    uint8_t buf[21] = { 0x10, 0x15, 0x21, 0x88, 0x02, 0x34, 0x12, 0x78,
                        0x56, 0x34, 0x12, 0x55, 0x55, 0x68, 0x6f, 0x67,
                        0x65, 0x68, 0x6f, 0x67, 0x65 };
#else
    uint8_t buf[22] = { 0x10, 0x16, 0x01, 0x88, 0x00, 0x01, 0x00, 0x01,
                        0x00, 0xcd, 0xab, 0x7e, 0xd4, 0x68, 0x6f, 0x67,
                        0x65, 0x68, 0x6f, 0x67, 0x65, 0x00 };
#endif

    ml7396_fifo_write(buf, 21);

    return 0;
}

int cmd_ml7396_status(int argc, char **argv)
{
    unsigned int channel;
    uint32_t status, enable;

    channel = ml7396_get_channel();

    enable = ml7396_get_interrupt_enable();
    status = ml7396_get_interrupt_status();

    printf("=== ML7396 status ===\n");
    printf(" - channel: %2lu\n", (unsigned long) channel);
    printf(" -- Interrupt --\n");
    printf("   - enable: 0x%08x\n", (unsigned int) enable);
    printf("   - status: 0x%08x\n", (unsigned int) status);

    return 0;
}
