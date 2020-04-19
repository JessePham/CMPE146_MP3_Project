#include "gpio2.h"
#include "FreeRTOS.h"
#include "lpc40xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void gpio2__set_as_input(uint8_t pin_num) { LPC_GPIO2->DIR &= ~(1 << pin_num); }
void gpio2__set_as_output(uint8_t pin_num) { LPC_GPIO2->DIR |= (1 << pin_num); }
void gpio2__set_high(uint8_t pin_num) { LPC_GPIO2->PIN |= (1 << pin_num); }
void gpio2__set_low(uint8_t pin_num) { LPC_GPIO2->PIN &= ~(1 << pin_num); }
void gpio2__set(uint8_t pin_num, bool high) {
  if (high) {
    LPC_GPIO2->PIN |= (1 << pin_num);
  } else {
    LPC_GPIO2->PIN &= ~(1 << pin_num);
  }
}
bool gpio2__get_level(uint8_t pin_num) {
  if (LPC_GPIO2->PIN & (1 << pin_num)) {
    printf("pin is high\n");
    return true;
  } else {
    printf("pin is low\n");
    return false;
  }
}
