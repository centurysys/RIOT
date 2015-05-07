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

//#include "api.h"
//#include "radio_cmds.h"
//#include "misc_cmds.h"
#include "ml7396_cmds.h"

#include "uart1.h"

extern void init_mma200(void);

static shell_command_t commands[] = {
#if 0
    { "apiRadioReset", "Reset Radio IC", cmd_api_RadioReset },
    { "apiSetOwnPanID", "set own PanID", cmd_api_SetOwnPanID },
    { "apiGetOwnPanID", "get own PanID", cmd_api_GetOwnPanID },
    { "apiSetOwnAddress", "set own Address", cmd_api_SetOwnAddress },
    { "apiGetOwnAddress", "get own Address", cmd_api_GetOwnAddress },
    { "apiSetChannel", "set Radio Channel", cmd_api_SetChannel },
    { "apiGetChannel", "get Radio Channel", cmd_api_GetChannel },
    { "apiSetOutputPower", "set OutputPower", cmd_api_SetOutputPower },
    { "apiGetOutputPower", "get OutputPower", cmd_api_GetOutputPower },
    { "radioStat", "get MMA-200 radio-status", cmd_get_radio_status },
    { "sendTest", "packet send test", cmd_sentPacketTest },
    { "recvTest", "packet recv test", cmd_receive_NBTest },
    { "vtimer", "vtimer_now", cmd_vtimer_test },
#endif
    { "ml7396_reads", "ml7396 reads", cmd_ml7396_reads },
    { "intstat", "ml7396 interrupt status/enable", cmd_ml7396_get_interrupt_status },
    { "ml7396_reset", "ml7396 reset", cmd_ml7396_reset },
    { "regwrite", "ml7396 write register", cmd_ml7396_regwrite },
    { "regread", "ml7396 read register", cmd_ml7396_regread },
    { "cca", "do CCA", cmd_cca },
    { "fifowrite", "write to FIFO test", cmd_fifo_write },
    { NULL, NULL, NULL }
};

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

    puts("Welcome to RIOT!");

    uart1_thread_init();

    shell_init(&shell, commands, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
