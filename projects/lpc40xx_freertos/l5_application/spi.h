#pragma once

#include <stdint.h>

void spi__init(uint32_t max_clock_mhz);
uint8_t spi__exchange_byte(uint8_t data_out);