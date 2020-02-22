#include "gpio_lab.h"

#include "lpc40xx.h"

static const LPC_GPIO_TypeDef *gpio_memory_map[] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4, LPC_GPIO5};

void gpio0__set_as_input(uint8_t port_num, uint8_t pin_num) {
  ((LPC_GPIO_TypeDef *)gpio_memory_map[port_num])->DIR &= ~(1 << pin_num);
}

void gpio0__set_as_output(uint8_t port_num, uint8_t pin_num) {
  ((LPC_GPIO_TypeDef *)gpio_memory_map[port_num])->DIR |= (1 << pin_num);
}

void gpio0__set_high(uint8_t port_num, uint8_t pin_num) {
  ((LPC_GPIO_TypeDef *)gpio_memory_map[port_num])->SET = (1 << pin_num);
}

void gpio0__set_low(uint8_t port_num, uint8_t pin_num) {
  ((LPC_GPIO_TypeDef *)gpio_memory_map[port_num])->CLR = (1 << pin_num);
}

void gpio0__set(uint8_t port_num, uint8_t pin_num, bool high) {
  if (high) {
    gpio0__set_high(port_num, pin_num);
  } else {
    gpio0__set_low(port_num, pin_num);
  }
}

bool gpio0__get_level(uint8_t port_num, uint8_t pin_num) {
  return (((LPC_GPIO_TypeDef *)gpio_memory_map[port_num])->PIN & (1 << pin_num));
}
