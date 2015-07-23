#include <stdio.h>
#include <unistd.h>

void dump_buffer(char *buf, size_t len)
{
    int i;

    for (i = 0; i < len; i++) {
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
