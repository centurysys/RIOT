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
#include <stdlib.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"

#define ENABLE_DEBUG (1)
#include "debug.h"


static int shell_readc(void)
{
    char c = 0;
    posix_read(uart0_handler_pid, &c, 1);
    return c;
}

static int shell_putchar(int c)
{
    putchar(c);
    fflush(0);

    return 1;
}

#if ENABLE_DEBUG
extern void ng_ml7396_regdump(int bank);

static int cmd_ng_ml7396_regdump(int argc, char **argv)
{
    int bank = 0;

    if (argc >= 2) {
        bank = atoi(argv[1]);

        if (bank < 0 || bank > 2)
            bank = 0;
    }

    ng_ml7396_regdump(bank);

    return 0;
}
#else
static int cmd_ng_ml7396_regdump(int argc, char **argv)
{
    puts("DEBUG disabled.");
    return 0;
}
#endif

static shell_command_t commands[] = {
    { "regdump", "ML7396B dump registers", cmd_ng_ml7396_regdump },
    { NULL, NULL, NULL }
};

int main(void)
{
    shell_t shell;
    posix_open(uart0_handler_pid, 0);

    puts("-- SA-200 console --");

    //shell_init(&shell, commands, UART0_BUFSIZE, shell_readc, shell_putchar);
    shell_init(&shell, commands, UART0_BUFSIZE, uart0_readc, uart0_putc);

    shell_run(&shell);
    return 0;
}
