#ifndef _CRC_H
#define _CRC_H

#include "kernel_types.h"

uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#endif
