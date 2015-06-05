/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"

static int shell_readc(void)
{
    char c = 0;
    posix_read(uart0_handler_pid, &c, 1);
    return c;
}

static void shell_putchar(int c)
{
    putchar(c);
    fflush(0);
}

int main(void)
{
    shell_t shell;
    posix_open(uart0_handler_pid, 0);

    puts("-- MMA-200 IAP console");

    shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
