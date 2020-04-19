#include "gpio_lab.h"
#include "FreeRTOS.h"
#include "lpc40xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void gpio0__set_as_input(uint8_t pin_num) { LPC_GPIO0->DIR &= ~(1 << pin_num); }
void gpio0__set_as_output(uint8_t pin_num) { LPC_GPIO0->DIR |= (1 << pin_num); }
void gpio0__set_high(uint8_t pin_num) { LPC_GPIO0->PIN |= (1 << pin_num); }
void gpio0__set_low(uint8_t pin_num) { LPC_GPIO0->PIN &= ~(1 << pin_num); }
void gpio0__set(uint8_t pin_num, bool high) {
  if (high) {
    LPC_GPIO0->PIN |= (1 << pin_num);
  } else {
    LPC_GPIO0->PIN &= ~(1 << pin_num);
  }
}
bool gpio0__get_level(uint8_t pin_num) {
  if (LPC_GPIO0->PIN & (1 << pin_num)) {
    printf("pin is high\n");
    return true;
  } else {
    printf("pin is low\n");
    return false;
  }
}
