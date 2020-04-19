#include "gpio1.h"
#include "FreeRTOS.h"
#include "lpc40xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void gpio1__set_as_input(uint8_t pin_num) { LPC_GPIO1->DIR &= ~(1 << pin_num); }
void gpio1__set_as_output(uint8_t pin_num) { LPC_GPIO1->DIR |= (1 << pin_num); }
void gpio1__set_high(uint8_t pin_num) { LPC_GPIO1->PIN |= (1 << pin_num); }
void gpio1__set_low(uint8_t pin_num) { LPC_GPIO1->PIN &= ~(1 << pin_num); }
void gpio1__set(uint8_t pin_num, bool high) {
  if (high) {
    LPC_GPIO1->PIN |= (1 << pin_num);
  } else {
    LPC_GPIO1->PIN &= ~(1 << pin_num);
  }
}
bool gpio1__get_level(uint8_t pin_num) {
  if (LPC_GPIO1->PIN & (1 << pin_num)) {
    printf("pin is high\n");
    return true;
  } else {
    printf("pin is low\n");
    return false;
  }
}
