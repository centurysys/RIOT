#include <stdio.h>


void dump_buffer(const char *buf, int buflen)
{
    int i;

    for (i = 0; i < buflen; i++) {
        if ((i % 16) == 0) {
            printf("%04x:", i);
        }

        printf(" %02x", buf[i]);

        if ((i % 16) == 15) {
            puts("");
        }
    }

    if ((i % 16) != 0) {
        puts("");
    }
}
