#include <stdio.h>

#include "board.h"
#include "periph/gpio.h"


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

struct conf_gpios {
    gpio_t dev;
    int shift;
};

static struct conf_gpios gpios[] = {
    { GPIO_5,  0 },
    { GPIO_11, 1 },
    { GPIO_0,  2 },
    { GPIO_0, -1 },
};

uint8_t get_config_pins(void)
{
    uint8_t conf = 0;
    int val;
    struct conf_gpios *gpio;

    for (gpio = &gpios[0]; gpio->shift >= 0; gpio++) {
        val = gpio_read(gpio->dev);
        conf |= (val != 0) << gpio->shift;
    }

    return conf;
}
