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
