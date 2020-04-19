#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lpc40xx.h"

void gpio2__set_as_input(uint8_t pin_num);
void gpio2__set_as_output(uint8_t pin_num);
void gpio2__set_high(uint8_t pin_num);
void gpio2__set_low(uint8_t pin_num);
void gpio2__set(uint8_t pin_num, bool high);
bool gpio2__get_level(uint8_t pin_num);
