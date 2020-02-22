#include "gpio_isr.h"

const unsigned long pin_lookup_table[32] = {
    [0] = 1,           [1] = 2,          [2] = 4,         [3] = 8,          [4] = 16,         [5] = 32,
    [6] = 64,          [7] = 128,        [8] = 256,       [9] = 512,        [10] = 1024,      [11] = 2048,
    [12] = 4096,       [13] = 8192,      [14] = 16384,    [15] = 32768,     [16] = 65536,     [17] = 131072,
    [18] = 262144,     [19] = 524288,    [20] = 1048576,  [21] = 2097152,   [22] = 4194304,   [23] = 8388608,
    [24] = 16777216,   [25] = 33554432,  [26] = 67108864, [27] = 134217728, [28] = 268435456, [29] = 536870912,
    [30] = 1073741824, [31] = 2147483648};

// Note: You may want another separate array for falling vs. rising edge callbacks
// static function_pointer_t gpio0_callbacks[32];
static function_pointer_t gpio0_callbacks_FE[32];
static function_pointer_t gpio0_callbacks_RE[32];

// find the pin with the interrupt
static int logic_that_you_will_write() {
  int pin_number = 0;
  // Check all 32 pins from port 0 to see if Status Register is HIGH
  for (int i = 0; i < 32; i++) {
    if (LPC_GPIOINT->IO0IntStatF == pin_lookup_table[i]) {
      pin_number = i;
    } else if (LPC_GPIOINT->IO0IntStatR == pin_lookup_table[i]) {
      pin_number = i;
    }
  }
  return pin_number;
}

static port_interrupt find_port(void) {
  port_interrupt p;

  for (int i = 0; i < 32; i++) {
    if (LPC_GPIOINT->IO0IntStatF == pin_lookup_table[i] || LPC_GPIOINT->IO0IntStatR == pin_lookup_table[i]) {
      p = PORT_0;
    } else {
      p = PORT_2;
    }
  }

  return p;
}

static gpio_interrupt_e rising_or_falling_edge(void) {
  gpio_interrupt_e level;
  for (int i = 0; i < 32; i++) {
    if (LPC_GPIOINT->IO0IntStatF == pin_lookup_table[i] || LPC_GPIOINT->IO2IntStatF == pin_lookup_table[i]) {
      level = GPIO_INTR__FALLING_EDGE;
    } else if (LPC_GPIOINT->IO0IntStatR == pin_lookup_table[i] || LPC_GPIOINT->IO2IntStatF == pin_lookup_table[i]) {
      level = GPIO_INTR__RISING_EDGE;
    }
  }

  return level;
}

static void clear_pin_interrupt(int pin_that_generated_interrupt) {
  LPC_GPIOINT->IO0IntClr |= (1 << pin_that_generated_interrupt);
}

void gpio0__attach_interrupt(uint32_t pin, gpio_interrupt_e interrupt_type, function_pointer_t callback) {
  // 1) Store the callback based on the pin at gpio0_callbacks
  // 2) Configure GPIO 0 pin for rising or falling edge

  if (interrupt_type == GPIO_INTR__FALLING_EDGE) {
    LPC_GPIOINT->IO0IntEnF |= (1 << pin);
    gpio0_callbacks_FE[pin] = callback;
  } else {
    LPC_GPIOINT->IO0IntEnR |= (1 << pin);
    gpio0_callbacks_RE[pin] = callback;
  }
}

// We wrote some of the implementation for you
void gpio0__interrupt_dispatcher(void) {
  // Check which pin generated the interrupt
  // const port_interrupt port_number = find_port();
  const gpio_interrupt_e edge = rising_or_falling_edge();
  const int pin_that_generated_interrupt = logic_that_you_will_write();

  function_pointer_t attached_user_handler;

  if (edge == GPIO_INTR__RISING_EDGE) {
    attached_user_handler = gpio0_callbacks_RE[pin_that_generated_interrupt];
  } else if (edge == GPIO_INTR__FALLING_EDGE) {
    attached_user_handler = gpio0_callbacks_FE[pin_that_generated_interrupt];
  }

  // Invoke the user registered callback, and then clear the interrupt
  attached_user_handler();
  clear_pin_interrupt(pin_that_generated_interrupt);
}