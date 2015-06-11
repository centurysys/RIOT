/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f2
 * @{
 *
 * @file
 * @brief       Low-level Flash-Mem driver implementation
 *
 * @author      
 *
 * @}
 */

#include <stdio.h>
#include <unistd.h>

#include "cpu.h"

#include "irq.h"
#include "flashrom.h"

#define INVALID_ADDRESS     (0xFF)

#define FLASH_KEY1 (0x45670123)
#define FLASH_KEY2 (0xcdef89ab)

static uint8_t addr_to_sector(uint32_t addr);
static void flash_lock(void);
static void flash_unlock(void);

static inline int is_flash_idle(void)
{
    int idle;

    if (FLASH->SR & FLASH_SR_BSY) {
        idle = -1;
    }
    else {
        idle = 0;
    }

    return idle;
}

static inline int wait_bsy_clear(int timeout_us)
{
    int res = -1;
    int interval = 10;

    while (timeout_us > 0) {
        if (is_flash_idle() == 0) {
            res = 0;
            break;
        }

        usleep(interval);
        timeout_us -= interval;
    }

    return res;
}

uint8_t flashrom_write(uint8_t *dst, const uint8_t *src, size_t size)
{
    int count, res = 1;

    if (is_flash_idle() != 0) {
        return 0;
    }

    flash_unlock();

    /* 8bit write */
    FLASH->CR &= ~(FLASH_CR_PSIZE_0 | FLASH_CR_PSIZE_1);

    for (count = 0; count < size; count++) {
        FLASH->CR |= FLASH_CR_PG;

        *dst++ = *src++;

        if (wait_bsy_clear(100) != 0) {
            printf("%s: TIMEDOUT\n", __FUNCTION__);
            res = 0;
            break;
        }
    }

    flash_lock();

    return res;
}

uint8_t flashrom_erase(uint8_t *addr)
{
    uint8_t sector;

    sector = addr_to_sector((uint32_t) addr);

    if (sector == INVALID_ADDRESS) {
        return 0;
    }

    if (is_flash_idle() != 0) {
        return 0;
    }

    flash_unlock();

    FLASH->CR |= FLASH_CR_SER | ((sector & 0x0f) << 3);
    FLASH->CR |= FLASH_CR_STRT;

    while (1) {
        if (is_flash_idle() == 0) {
            break;
        }

        usleep(1000);
    }

    flash_lock();

    return 1;
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_unlock(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

static uint8_t addr_to_sector(uint32_t addr)
{
    if ((addr >= 0x08000000) && (addr < 0x08004000)) {
        return 0;
    }
    else if ((addr >= 0x08004000) && (addr < 0x08008000)) {
        return 1;
    }
    else if ((addr >= 0x08008000) && (addr < 0x0800c000)) {
        return 2;
    }
    else if ((addr >= 0x0800c000) && (addr < 0x08010000)) {
        return 3;
    }
    else if ((addr >= 0x08010000) && (addr < 0x08020000)) {
        return 4;
    }
    else if ((addr >= 0x08020000) && (addr < 0x08040000)) {
        return 5;
    }
    else if ((addr >= 0x08040000) && (addr < 0x08060000)) {
        return 6;
    }
    else if ((addr >= 0x08060000) && (addr < 0x08080000)) {
        return 7;
    }

    return INVALID_ADDRESS;
}

