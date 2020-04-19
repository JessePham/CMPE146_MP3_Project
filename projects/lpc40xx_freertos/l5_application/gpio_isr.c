#include "gpio_isr.h"
#include "lpc40xx.h"
#include <stdint.h>
#include <stdio.h>

static function_pointer_t gpio0_callbacks_f[32];
static function_pointer_t gpio0_callbacks_r[32];
void gpio0__attach_interrupt(uint32_t pin, gpio_interrupt_e interrupt_type, function_pointer_t callback) {
  if (interrupt_type == GPIO_INTR__RISING_EDGE) {
    gpio0_callbacks_r[pin] = callback;
    LPC_GPIOINT->IO0IntEnR |= (1 << pin);
  }
  if (interrupt_type == GPIO_INTR__FALLING_EDGE) {
    LPC_GPIOINT->IO0IntEnF |= (1 << pin);
    gpio0_callbacks_f[pin] = callback;
  }
}

void gpio0__interrupt_dispatcher(void) {
  int pin = 0;
  for (int i = 0; i < 32; i++) {
    if (LPC_GPIOINT->IO0IntStatR & (1 << i)) {
      pin = i;
    } else if (LPC_GPIOINT->IO0IntStatF & (1 << i)) {
      pin = i;
    }
  }
  const int pin_that_generated_interrupt = pin;
  function_pointer_t attached_user_handler;
  if (LPC_GPIOINT->IO0IntStatR & (1 << pin_that_generated_interrupt)) {
    attached_user_handler = gpio0_callbacks_r[pin_that_generated_interrupt];
  }
  if (LPC_GPIOINT->IO0IntStatF & (1 << pin_that_generated_interrupt)) {
    attached_user_handler = gpio0_callbacks_f[pin_that_generated_interrupt];
  }
  LPC_GPIOINT->IO0IntClr = (1 << pin_that_generated_interrupt);
  attached_user_handler();
}