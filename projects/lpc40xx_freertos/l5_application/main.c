/*================== Lab 04 ADC + PWN ==================*/
#if 1

#include "pwm1.h"

#include "FreeRTOS.h"
#include "task.h"

void pin_configure_pwm_channel_as_io_pin() {
  LPC_IOCON->P2_0 &= ~(7 << 0);
  LPC_IOCON->P2_0 |= (1 << 0);
}

void pwm_task(void *p) {
  pwm1__init_single_edge(1000);
  
  // Locate a GPIO pin that a PWM channel will control
  // NOTE You can use gpio__construct_with_function() API from gpio.h
  // TODO Write this function yourself
  pin_configure_pwm_channel_as_io_pin();
  
  // We only need to set PWM configuration once, and the HW will drive
  // the GPIO at 1000Hz, and control set its duty cycle to 50%
  pwm1__set_duty_cycle(PWM1__2_0, 50);
  
  // Continue to vary the duty cycle in the loop
  uint8_t percent = 0;
  while (1) {
    pwm1__set_duty_cycle(PWM1__2_0, percent);
    
    if (++percent > 100) { 
      percent = 0; 
    }
    
    vTaskDelay(100);
  }
}

void main(void) {
  xTaskCreate(pwm_task, "PWM_Task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}

#endif

#if 0
// Part 0

#include "pwm1.h"

#include "FreeRTOS.h"
#include "delay.h"
#include "task.h"

void pin_configure_pwm_channel_as_io_pin() {
  LPC_IOCON->P2_0 &= ~(7 << 0);
  LPC_IOCON->P2_0 |= (1 << 0);
}

// void pwm_task(void *p) {
//   pwm1__init_single_edge(1000);

//   // Locate a GPIO pin that a PWM channel will control
//   // You can use gpio__construct_with_function() API from gpio.h
//   pin_configure_pwm_channel_as_io_pin(); // Write this function yourself

//   // We only need to set PWM configuration once, and the HW will drive
//   // the GPIO at 1000Hz, and control set its duty cycle to 50%
//   pwm1__set_duty_cycle(PWM1__2_0, 50);

//   // Continue to vary the duty cycle in the loop
//   uint8_t percent = 0;
//   while (1) {
//     pwm1__set_duty_cycle(PWM1__2_0, percent);

//     if (++percent > 100) {
//       percent = 0;
//     }
//     vTaskDelay(100);
//   }
// }

void main(void) {
  // xTaskCreate(pwm_task, "PWM_Task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  // vTaskStartScheduler();
  pin_configure_pwm_channel_as_io_pin();
  pwm1__init_single_edge(1000);

  // Locate a GPIO pin that a PWM channel will control
  // You can use gpio__construct_with_function() API from gpio.h
  pin_configure_pwm_channel_as_io_pin(); // Write this function yourself

  // We only need to set PWM configuration once, and the HW will drive
  // the GPIO at 1000Hz, and control set its duty cycle to 50%
  pwm1__set_duty_cycle(PWM1__2_0, 100);

  delay__ms

  // Continue to vary the duty cycle in the loop
  uint8_t percent = 0;
  while (1) {
    pwm1__set_duty_cycle(PWM1__2_0, percent);

    if (++percent > 100) {
      percent = 0;
    }
    delay__ms(100);
  }
}

#endif

/*================== Lab 03 Interrupts ==================*/
#if 0
// Part 2

#include "FreeRTOS.h"
#include "delay.h"
#include "gpio_isr.h"
#include "lpc_peripherals.h"
#include "semphr.h"
#include "stdio.h"

static SemaphoreHandle_t switch_pressed_signal;

// Objective of the assignment is to create a clean API to register sub-interrupts like so:
void pin30_isr(void) {
  xSemaphoreGiveFromISR(switch_pressed_signal, NULL);
  fprintf(stderr, "Pin30isr\n");
}
void pin29_isr(void) {
  xSemaphoreGiveFromISR(switch_pressed_signal, NULL);
  fprintf(stderr, "Pin29isr\n");
}

void gpio_interrupt(void) { gpio0__interrupt_dispatcher(); }

void sleep_on_sem_task(void *p) {
  LPC_IOCON->P2_3 &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_IOCON->P2_3 |= (1 << 3);
  LPC_GPIO2->DIR |= (1 << 3);

  while (1) {
    if (xSemaphoreTake(switch_pressed_signal, 1000)) {
      LPC_GPIO2->SET |= (1 << 3);
      vTaskDelay(500);
      LPC_GPIO2->CLR |= (1 << 3);
      vTaskDelay(500);
    }
  }
}

// Example usage:
void main(void) {
  switch_pressed_signal = xSemaphoreCreateBinary(); // Create your binary semaphore
  LPC_IOCON->P1_18 &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_IOCON->P1_18 |= (1 << 3);
  LPC_GPIO1->DIR |= (1 << 18);

  gpio0__attach_interrupt(30, GPIO_INTR__RISING_EDGE, pin30_isr);
  gpio0__attach_interrupt(30, GPIO_INTR__FALLING_EDGE, pin29_isr);
  gpio0__attach_interrupt(29, GPIO_INTR__FALLING_EDGE, pin29_isr);
  lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__GPIO, gpio_interrupt);

  NVIC_EnableIRQ(GPIO_IRQn);

  xTaskCreate(sleep_on_sem_task, "sem", (512U * 4) / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}

#endif
#if 0
// Part 1

#include "FreeRTOS.h"
#include "lpc_peripherals.h"
#include "semphr.h"
#include <stdio.h>

#include "lpc40xx.h"

void gpio_interrupt(void);
void sleep_on_sem_task(void *);
void configure_your_gpio_interrupt(void);
void clear_gpio_interrupt(void);

static SemaphoreHandle_t switch_pressed_signal;

void main(void) {
  switch_pressed_signal = xSemaphoreCreateBinary(); // Create your binary semaphore

  configure_your_gpio_interrupt(); // TODO: Setup interrupt by re-using code from Part 0
  lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__GPIO, gpio_interrupt);
  NVIC_EnableIRQ(GPIO_IRQn); // Enable interrupt gate for the GPIO

  xTaskCreate(sleep_on_sem_task, "sem", (512U * 4) / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}

void configure_your_gpio_interrupt(void) {
  LPC_GPIOINT->IO0IntEnF |= (1 << 30);
}

void clear_gpio_interrupt(void) { LPC_GPIOINT->IO0IntClr |= (1 << 30); }

// Warning: You can only use uart_printf__polled() inside of an ISR
void gpio_interrupt(void) {
  xSemaphoreGiveFromISR(switch_pressed_signal, NULL);
  clear_gpio_interrupt();
}

void sleep_on_sem_task(void *p) {
  LPC_IOCON->P2_3 &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_GPIO2->DIR |= (1 << 3);

  while (1) {
    if (xSemaphoreTake(switch_pressed_signal, 1000)) {
      LPC_GPIO2->SET |= (1 << 3);
      vTaskDelay(500);
      LPC_GPIO2->CLR |= (1 << 3);
      vTaskDelay(500);
    }
  }
}

#endif
#if 0
// Part 0
#include "delay.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include "stdio.h"

void gpio_interrupt(void);

// Step 1:
void main(void) {
  LPC_IOCON->P0_30 |= (1 << 3);
  LPC_GPIOINT->IO0IntEnF |= (1 << 30);

  LPC_IOCON->P2_3 &= ~((1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_GPIO2->DIR |= (1 << 3);

  lpc_peripheral__enable_interrupt(LPC_PERIPHERAL__GPIO, gpio_interrupt);

  NVIC_EnableIRQ(GPIO_IRQn);

  while (1) {
    LPC_GPIO2->CLR |= (1 << 3);
    delay__ms(500);
    LPC_GPIO2->SET |= (1 << 3);
    delay__ms(500);
  }
}

// Step 2:
void gpio_interrupt(void) {
  fprintf(stderr, "ISR Has been Activated!\n");
  LPC_GPIOINT->IO0IntClr |= (1 << 30);
}
#endif

/*================== Lab 02 GPIO ==================*/
#if 0
// Lab 02 Part 0
void led_task(void *pvParameters) {
  // Choose one of the onboard LEDS by looking into schematics and write code for the below
  LPC_IOCON->P2_3 = 0;
  // 1) Set the DIR register bit for the LED port pin
  LPC_GPIO2->DIR |= (1 << 3);

  while (true) {
    // 2) Use CLR register to turn the LED ON (led may be active low)
    LPC_GPIO2->CLR |= (1 << 3);
    vTaskDelay(500);

    // 3) Use SET register to turn the LED OFF
    LPC_GPIO2->SET |= (1 << 3);
    vTaskDelay(500);
  }
}

int main(void) {
  // Create FreeRTOS LED task
  xTaskCreate(led_task, "led", 1024 / sizeof(void *), NULL, PRIORITY_LOW, NULL);

  vTaskStartScheduler();
  return 0;
}
#endif

#if 0
// Lab 02 Part 1

void button_task(void *pvParameters) {

  while (true) {
    if (gpio0__get_level(1, 19)) {
      printf("Button Pressed!\n");
      vTaskDelay(1000);
    }
  }
}

int main(void) {
  LPC_IOCON->P0_0 &= ~((1 << 2) | (1 << 1) | (1 << 0));
  LPC_IOCON->P1_19 &= ~((1 << 2) | (1 << 1) | (1 << 0));

  gpio0__set_as_input(1, 19);
  gpio0__set_as_output(0, 0);

  gpio0__set_high(0, 0);
  printf("Set High: %d\n", gpio0__get_level(0, 0));

  gpio0__set_low(0, 0);
  printf("Set Low: %d\n", gpio0__get_level(0, 0));

  gpio0__set(0, 0, true);
  printf("Set True: %d\n", gpio0__get_level(0, 0));

  gpio0__set(0, 0, false);
  printf("Set False: %d\n", gpio0__get_level(0, 0));

  xTaskCreate(button_task, "button", 1024 / sizeof(void *), NULL, PRIORITY_LOW, NULL);

  vTaskStartScheduler();

  return 0;
}
#endif

#if 0
// Lab 02 Part 2
typedef struct {
  uint8_t port;
  uint8_t pin;
} port_pin_s;

void led_task(void *task_parameter) {
  // Type-cast the parameter that was passed from xTaskCreate()
  port_pin_s *led = (port_pin_s *)(task_parameter);

  gpio0__set_as_output(led->port, led->pin);

  while (true) {
    // TODO: insert code here to blink an LED
    gpio0__set_high(led->port, led->pin);
    vTaskDelay(500);
    gpio0__set_low(led->port, led->pin);
    vTaskDelay(500);
  }
}

int main(void) {
  // TODO:
  // Create two tasks using led_task() function
  // Pass each task its own parameter:
  // This is static such that these variables will be allocated in RAM and not go out of scope
  static port_pin_s led0 = {2, 3};
  static port_pin_s led1 = {1, 18};

  xTaskCreate(led_task, "led_task0", 1024 / sizeof(void *), &led0, PRIORITY_LOW, NULL);
  xTaskCreate(led_task, "led_task1", 1024 / sizeof(void *), &led1, PRIORITY_LOW, NULL);

  vTaskStartScheduler();
  return 0;
}

#endif

#if 0
// Lab 02 Part 3

typedef struct {
  uint8_t port;
  uint8_t pin;
} port_pin_s;

static SemaphoreHandle_t switch_press_indication;
static port_pin_s swch = {1, 19};

void led_task(void *task_parameter) {
  port_pin_s *led = (port_pin_s *)task_parameter;y;

  LPC_IOCON->P2_3 &= ~((1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
  gpio0__set_as_output(led->port, led->pin);

  while (true) {
    if (xSemaphoreTake(switch_press_indication, 1000)) {
      // TODO: Blink the LED
      while (gpio0__get_level(swch.port, swch.pin)) {
      }

      gpio0__set_high(led->port, led->pin);
      vTaskDelay(100);
      gpio0__set_low(led->port, led->pin);
      vTaskDelay(100);
    } else {
      puts("Timeout: No switch press indication for 1000ms");
    }
  }
}

void switch_task(void *task_parameter) {
  port_pin_s *sw = (port_pin_s *)task_parameter;

  LPC_IOCON->P1_19 &= ~((1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_IOCON->P1_19 |= (1 << 3);
  gpio0__set_as_input(sw->port, sw->pin);

  while (true) {
    // TODO: If switch pressed, set the binary semaphore
    if (gpio0__get_level(sw->port, sw->pin)) {
      printf("Button Press\n");
      xSemaphoreGive(switch_press_indication);
    }
    vTaskDelay(100);
  }
}

int main(void) {
  switch_press_indication = xSemaphoreCreateBinary();

  // Hint: Use on-board LEDs first to get this logic to work
  //       After that, you can simply switch these parameters to off-board LED and a switch
  // external value: 1, 23
  static port_pin_s led = {2, 3}; // external value: 2, 4

  xTaskCreate(switch_task, "switch", 1024 / sizeof(void *), &swch, PRIORITY_LOW, NULL);
  xTaskCreate(led_task, "led", 1024 / sizeof(void *), &led, PRIORITY_LOW, NULL);

  vTaskStartScheduler();

  return 0;
}

#endif

#if 0
// Lab 02 Extra Credit

typedef struct {
  uint8_t port;
  uint8_t pin;
} port_pin_s;

static SemaphoreHandle_t switch_press_indication;

void led_task(void *task_parameter) {
  port_pin_s *led_arr = (port_pin_s *)task_parameter;

  LPC_IOCON->P2_3 &= ~(7 << 3);
  LPC_IOCON->P2_1 &= ~(7 << 3);
  LPC_IOCON->P2_2 &= ~(7 << 3);
  LPC_IOCON->P2_0 &= ~(7 << 3);

  for (int i = 0; i < 4; i++) {
    gpio0__set_as_output(led_arr[i].port, led_arr[i].pin);
  }

  while (true) {
    if (xSemaphoreTake(switch_press_indication, 1000)) {
      for (int i = 0; i < 4; i++) {
        gpio0__set_high(led_arr[i].port, led_arr[i].pin);
        vTaskDelay(100);
      }

      int stopper = 0;
      for (int j = 0; j < 4; j++) {
        for (int i = 3; i >= stopper; i--) {
          if (i < 3) {
            gpio0__set_low(led_arr[i + 1].port, led_arr[i + 1].pin);
          }
          gpio0__set_high(led_arr[i].port, led_arr[i].pin);
          vTaskDelay(200);
        }
        stopper++;
      }

      for (int i = 3; i >= 1; i--) {
        for (int k = 0; k < 2; k++) {
          gpio0__set_low(led_arr[0].port, led_arr[0].pin);
          vTaskDelay(100);
          gpio0__set_high(led_arr[0].port, led_arr[0].pin);
          vTaskDelay(100);
        }

        gpio0__set_low(led_arr[0].port, led_arr[0].pin);
        vTaskDelay(200);

        gpio0__set_low(led_arr[i].port, led_arr[i].pin);
        gpio0__set_high(led_arr[0].port, led_arr[0].pin);
        vTaskDelay(200);
      }

      // All Flash On and Off x2
      for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 4; i++) {
          gpio0__set_low(led_arr[i].port, led_arr[i].pin);
        }
        vTaskDelay(300);
        for (int i = 0; i < 4; i++) {
          gpio0__set_high(led_arr[i].port, led_arr[i].pin);
        }
        vTaskDelay(300);
      }

      for (int i = 0; i < 4; i++) {
        gpio0__set_low(led_arr[i].port, led_arr[i].pin);
      }

      vTaskDelay(100);
    } else {
      puts("Timeout: No switch press indication for 1000ms");
    }
  }
}

void switch_task(void *task_parameter) {
  port_pin_s *sw = (port_pin_s *)task_parameter;

  LPC_IOCON->P1_23 &= ~((1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
  LPC_IOCON->P1_23 |= (1 << 3);
  gpio0__set_as_input(sw->port, sw->pin);

  while (true) {
    if (gpio0__get_level(sw->port, sw->pin)) {
      xSemaphoreGive(switch_press_indication);
    }
    vTaskDelay(500); // originally 100ms
  }
}

int main(void) {
  switch_press_indication = xSemaphoreCreateBinary();

  static port_pin_s sw = {1, 23};
  static port_pin_s led_arr[] = {{2, 4}, {2, 2}, {2, 1}, {2, 0}};

  xTaskCreate(switch_task, "switch", 1024 / sizeof(void *), &sw, PRIORITY_LOW, NULL);
  xTaskCreate(led_task, "led", 1024 / sizeof(void *), &led_arr, PRIORITY_LOW, NULL);

  vTaskStartScheduler();

  return 0;
}
#endif

#if 0
#include "FreeRTOS.h"
#include "gpio_lab.h"
#include "lpc40xx.h"
#include "semphr.h"
#include "stdbool.h"
#include "stdio.h"
#include "task.h"

//Lab 01 FreeRTOS Tasks
static void task_one(void *task_parameter);
static void task_two(void *task_parameter);

int main(void) {
  const uint32_t STACK_SIZE = 4096 / sizeof(void *);

  xTaskCreate(task_one, "TaskOneCode", STACK_SIZE, NULL, PRIORITY_HIGH, NULL);
  xTaskCreate(task_two, "TaskTwoCode", STACK_SIZE, NULL, PRIORITY_HIGH, NULL);

  vTaskStartScheduler();
}

static void task_one(void *task_parameter) {
  while (1) {
    // Read existing main.c regarding when we should use fprintf(stderr...) in place of printf()
    // For this lab, we will use fprintf(stderr, ...)
    fprintf(stderr, "AAAAAAAAAAAA");

    // Sleep for 100ms
    vTaskDelay(100);
  }
}

static void task_two(void *task_parameter) {
  while (1) {
    fprintf(stderr, "bbbbbbbbbbbb");
    vTaskDelay(100);
  }
}
#endif