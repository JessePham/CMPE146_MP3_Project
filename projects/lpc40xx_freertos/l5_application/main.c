/*================== Lab 05 SPI Flash Interface ==================*/
#if 1
// Lab 5.0
#include "FreeRTOS.h"
#include "gpio.h"
#include "spi.h"
#include "task.h"
#include <stdio.h>

// TODO: Implement Adesto flash memory CS signal as a GPIO driver
void adesto_cs(void);
void adesto_ds(void);

// TODO: Study the Adesto flash 'Manufacturer and Device ID' section
typedef struct {
  uint8_t manufacturer_id;
  uint8_t device_id_1;
  uint8_t device_id_2;
  uint8_t extended_device_id;
} adesto_flash_id_s;

void adesto_cs(void) {
  LPC_GPIO1->PIN &= ~(1 << 10); // Set *CS LOW
}

void adesto_ds(void) {
  LPC_GPIO1->PIN |= (1 << 10); // Set *CS HIGH
  LPC_GPIO1->PIN |= (1 << 10);
}

// TODO: Implement the code to read Adesto flash memory signature
// TODO: Create struct of type 'adesto_flash_id_s' and return it
adesto_flash_id_s adesto_read_signature(void) {
  adesto_flash_id_s data = {0};

  adesto_cs();

  LPC_SSP2->DR = 0x9F; // Sending the Opcode
  data.manufacturer_id = spi__exchange_byte(0xFF);
  data.device_id_1 = spi__exchange_byte(0xFF);
  data.device_id_2 = spi__exchange_byte(0xFF);
  data.extended_device_id = spi__exchange_byte(0xFF);
  adesto_ds();

  return data;
}

void todo_configure_your_spi_pin_functions(void) {
  // Setting up SCK
  gpio__construct_with_function(GPIO__PORT_1, 0, GPIO__FUNCTION_4);
  // Setting up MOSI
  gpio__construct_with_function(GPIO__PORT_1, 1, GPIO__FUNCTION_4);
  // Setting up MISO
  gpio__construct_with_function(GPIO__PORT_1, 4, GPIO__FUNCTION_4);
  // Setting up *CS
  gpio__construct_with_function(GPIO__PORT_1, 10, GPIO__FUNCITON_0_IO_PIN);
  gpio__construct_as_output(GPIO__PORT_1, 10);
}

void spi_task(void *p) {
  const uint32_t spi_clock_mhz = 24;
  spi__init(spi_clock_mhz);

  // From the LPC schematics pdf, find the pin numbers connected to flash memory
  // Read table 84 from LPC User Manual and configure PIN functions for SPI2 pins
  // You can use gpio__construct_with_function() API from gpio.h
  //
  // Note: Configure only SCK2, MOSI2, MISO2.
  // CS will be a GPIO output pin(configure and setup direction)
  todo_configure_your_spi_pin_functions();

  while (1) {
    adesto_flash_id_s id = adesto_read_signature();
    // TODO: printf the members of the 'adesto_flash_id_s' struct

    printf("\nManufacturer ID: %x\nDevice ID_1: %x\nDevice ID_2: %x\nExtended Device ID: %x\n", id.manufacturer_id,
           id.device_id_1, id.device_id_2, id.extended_device_id);

    vTaskDelay(500);
  }
}

void main(void) {
  xTaskCreate(spi_task, "SPI_TASK", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}

#endif

/*================== Lab 04 ADC + PWN ==================*/
#if 0
// Lab 4.2

#include "adc.h"
#include "pwm1.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>

// This is the queue handle we will need for the xQueue Send/Receive API
static QueueHandle_t adc_to_pwm_task_queue;

void pin_configure_adc_channel_as_io_pin(void) {
  LPC_IOCON->P1_31 &= ~(7 << 0);
  LPC_IOCON->P1_31 |= (3 << 0);
  LPC_IOCON->P1_31 &= ~(3 << 3);
  LPC_IOCON->P1_31 |= (1 << 3);
  LPC_IOCON->P1_31 &= ~(1 << 7);
}

void adc_task(void *p) {
  // NOTE: Reuse the code from Part 1

  int adc_reading = 0; // Note that this 'adc_reading' is not the same variable as the one from adc_task

  adc__initialize();
  adc__enable_burst_mode();
  pin_configure_adc_channel_as_io_pin(); // You need to write this function

  while (1) {

    adc_reading = adc__get_channel_reading_with_burst_mode(ADC__CHANNEL_5);
    if (xQueueSend(adc_to_pwm_task_queue, &adc_reading, 100)) {
      fprintf(stderr, "Voltage: %.3fV\nADC: %d\n", ((adc_reading * 3.3) / 4096), adc_reading);
    } else {
      puts("Nothing Sent");
    }

    vTaskDelay(100);
  }
}

void pin_configure_pwm_channel_as_io_pin() {
  LPC_IOCON->P2_0 &= ~(7 << 0);
  LPC_IOCON->P2_0 |= (1 << 0);
  LPC_IOCON->P2_1 &= ~(7 << 0);
  LPC_IOCON->P2_1 |= (1 << 0);
  LPC_IOCON->P2_2 &= ~(7 << 0);
  LPC_IOCON->P2_2 |= (1 << 0);
}

void pwm_task(void *p) {
  // NOTE: Reuse the code from Part 0
  int adc_reading = 0;

  pwm1__init_single_edge(1000);
  pin_configure_pwm_channel_as_io_pin();

  while (1) {
    // Implement code to receive potentiometer value from queue
    if (xQueueReceive(adc_to_pwm_task_queue, &adc_reading, 100)) {
      if (adc_reading <= 10) {
        pwm1__set_duty_cycle(PWM1__2_0, 0);
        pwm1__set_duty_cycle(PWM1__2_1, 0);
        pwm1__set_duty_cycle(PWM1__2_2, 0);
      } else if (adc_reading > 10 && adc_reading < 500) { // Red
        pwm1__set_duty_cycle(PWM1__2_0, adc_reading);
        pwm1__set_duty_cycle(PWM1__2_1, 0);
        pwm1__set_duty_cycle(PWM1__2_2, 0);
      } else if (adc_reading >= 500 && adc_reading < 1000) { // Magenta
        pwm1__set_duty_cycle(PWM1__2_0, 1000 - adc_reading);
        pwm1__set_duty_cycle(PWM1__2_1, 0);
        pwm1__set_duty_cycle(PWM1__2_2, adc_reading - 500);
      } else if (adc_reading >= 1000 && adc_reading < 1500) { // Blue
        pwm1__set_duty_cycle(PWM1__2_0, 0);
        pwm1__set_duty_cycle(PWM1__2_1, 0);
        pwm1__set_duty_cycle(PWM1__2_2, adc_reading);
      } else if (adc_reading >= 1500 && adc_reading < 2000) { // Teal
        pwm1__set_duty_cycle(PWM1__2_0, 0);
        pwm1__set_duty_cycle(PWM1__2_1, adc_reading - 1500);
        pwm1__set_duty_cycle(PWM1__2_2, 2000 - adc_reading);
      } else if (adc_reading >= 2000 && adc_reading < 2500) { // Green
        pwm1__set_duty_cycle(PWM1__2_0, 0);
        pwm1__set_duty_cycle(PWM1__2_1, adc_reading);
        pwm1__set_duty_cycle(PWM1__2_2, 0);
      } else if (adc_reading >= 2500 && adc_reading < 3500) { // Yellow
        pwm1__set_duty_cycle(PWM1__2_0, adc_reading - 2500);
        pwm1__set_duty_cycle(PWM1__2_1, adc_reading);
        pwm1__set_duty_cycle(PWM1__2_2, 0);
      } else if (adc_reading >= 3500 && adc_reading < 4000) { // White
        pwm1__set_duty_cycle(PWM1__2_0, adc_reading);
        pwm1__set_duty_cycle(PWM1__2_1, adc_reading);
        pwm1__set_duty_cycle(PWM1__2_2, adc_reading - 3500);
      }
    }
    // fprintf(stderr, "\nM0: %li\nM1: %li\nM2: %li\nM3: %li\n", LPC_PWM1->MR0, LPC_PWM1->MR1, LPC_PWM1->MR2,
    //         LPC_PWM1->MR3);
  }
}

void main(void) {
  // Queue will only hold 1 integer
  adc_to_pwm_task_queue = xQueueCreate(1, sizeof(int));

  xTaskCreate(adc_task, "ADC_Task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(pwm_task, "PWM_Task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}
#endif

#if 0
// Lab 4.1

#include "adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void pin_configure_adc_channel_as_io_pin(void) {
  LPC_IOCON->P1_31 &= ~(7 << 0);
  LPC_IOCON->P1_31 |= (3 << 0);
  LPC_IOCON->P1_31 &= ~(3 << 3);
  LPC_IOCON->P1_31 |= (1 << 3);
  LPC_IOCON->P1_31 &= ~(1 << 7);
}

void adc_task(void *p) {
  adc__initialize();

  // This is the function you need to add to adc.h
  // You can configure burst mode for just the channel you are using
  adc__enable_burst_mode();

  // Configure a pin, such as P1.31 with FUNC 011 to route this pin as ADC channel 5
  // You can use gpio__construct_with_function() API from gpio.h
  pin_configure_adc_channel_as_io_pin(); // You need to write this function

  while (1) {
    // Get the ADC reading using adc__get_adc_value() and print it
    // TODO: You need to write the implementation of this function
    const uint16_t adc_value = adc__get_channel_reading_with_burst_mode(ADC__CHANNEL_5);

    fprintf(stderr, "%i\n", adc_value);

    vTaskDelay(100);
  }
}

void main(void) {
  xTaskCreate(adc_task, "ADC_Task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  vTaskStartScheduler();
}

#endif

#if 0
// Lab 4.0
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