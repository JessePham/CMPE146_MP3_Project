#pragma once

#include "lpc40xx.h"
#include <stdint.h>

void i2c2__slave_init(uint8_t slave_address_to_respond_to) {
  LPC_I2C2->ADR0 |= (slave_address_to_respond_to);
  LPC_I2C2->CONSET |= (0x44);
}