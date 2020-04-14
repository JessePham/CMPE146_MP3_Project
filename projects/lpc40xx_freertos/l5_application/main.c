#if 1
#include "ff.h"
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "cli_handlers.h"
#include "queue.h"
#include "sj2_cli.h"
#include "string.h"

void mp3_reader_task(void *p);
void mp3_player_task(void *p);
typedef char songname_t[32];
QueueHandle_t Q_songname;
QueueHandle_t Q_songdata;
FIL file; // File handle
int main(void) {
  Q_songname = xQueueCreate(1, sizeof(songname_t));
  Q_songdata = xQueueCreate(1, 512);
  xTaskCreate(mp3_reader_task, "mp3_read", 2048 / sizeof(void *), NULL, 1, NULL);
  xTaskCreate(mp3_player_task, "mp3_play", 2048 / sizeof(void *), NULL, 1, NULL);
  sj2_cli__init();
  vTaskStartScheduler();
  return 0;
}
void mp3_reader_task(void *p) {
  songname_t name;
  char bytes_512[512];
  UINT bytes_read = 0;
  while (1) {
    if (xQueueReceive(Q_songname, &name[0], portMAX_DELAY)) {
      printf("Song recieved: %s\n", name);
      FRESULT read_file = f_open(&file, &name[0], FA_READ);
      if (FR_OK == read_file) {
        while (!f_eof(&file)) {
          if (FR_OK == f_read(&file, bytes_512, 512, &bytes_read)) {
            printf("\nSong Data Sent: \n");
            for (int i = 0; i < sizeof(bytes_512); i++) {
              // printf("%x ", bytes_512[i]);
            }
            xQueueSend(Q_songdata, &bytes_512[0], portMAX_DELAY);
            vTaskDelay(300);
          } else {
            printf("ERROR: Failed to read data to file\n");
          }
        }
      } else {
        printf("file doesnt exist\n");
      }
      f_close(&file);
    }
  }
}
void mp3_player_task(void *p) {
  char bytes_512[512];
  while (1) {
    if (xQueueReceive(Q_songdata, &bytes_512[0], portMAX_DELAY)) {
      printf("Song Data Received: \n");
      for (int i = 0; i < sizeof(bytes_512); i++) {
        printf("%x ", bytes_512[i]);
        //   // putchar(bytes_512[i]);
      }
    }
  }
}

app_cli_status_e cli__mp3_play(app_cli__argument_t argument, sl_string_t user_input_minus_command_name,
                               app_cli__print_string_function cli_output) {
  // user_input_minus_command_name is actually a 'char *' pointer type
  // We tell the Queue to copy 32 bytes of songname from this location
  xQueueSend(Q_songname, user_input_minus_command_name, portMAX_DELAY);

  printf("Sent %s over to the Q_songname\n", user_input_minus_command_name);
  return APP_CLI_STATUS__SUCCESS;
}
#endif

#if 0
#include "ff.h"
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "cli_handlers.h"
#include "queue.h"
#include "sj2_cli.h"
typedef char songname_t[16];

QueueHandle_t Q_songname;
QueueHandle_t Q_songdata;

// Reader tasks receives song-name over Q_songname to start reading it
void mp3_reader_task(void *p) {
  songname_t name;
  char bytes_512[512];

  const char *filename = "file.txt";

  FIL file;

  while (1) {
    xQueueReceive(Q_songname, &name[0], portMAX_DELAY);
    fprintf(stderr, "Received song to play: %s\n", name);

    FRESULT result = f_open(&file, filename, (FA_READ | FA_OPEN_EXISTING));
    if (FR_OK != result) {
      fprintf(stderr, "Error opening file %s", filename);
    } else {
      while (FR_OK == result) {
        // read_from_file(bytes_512); TO DO!!!!
        xQueueSend(Q_songdata, &bytes_512[0], portMAX_DELAY);
      }
      f_close(&file);
    }
  }
}

bool mp3_decoder_needs_data() { return true; }
void spi_send_to_mp3_decoder(char bytes) {}

// Player task receives song data over Q_songdata to send it to the MP3 decoder
void mp3_player_task(void *p) {
  char bytes_512[512];

  while (1) {
    if (xQueueReceive(Q_songdata, &bytes_512[0], portMAX_DELAY)){
      fprintf(stderr, "Received!!!\n");
    }
    for (int i = 0; i < sizeof(bytes_512); i++) {
      while (!mp3_decoder_needs_data()) {
        vTaskDelay(1);
      }

      spi_send_to_mp3_decoder(bytes_512[i]);
    }
  }
}

// CLI needs access to the QueueHandle_t where you can queue the song name
// One way to do this is to declare 'QueueHandle_t' in main() that is NOT static
// and then extern it here

app_cli_status_e cli__mp3_play(app_cli__argument_t argument, sl_string_t user_input_minus_command_name,
                               app_cli__print_string_function cli_output) {
  // user_input_minus_command_name is actually a 'char *' pointer type
  // We tell the Queue to copy 32 bytes of songname from this location
  xQueueSend(Q_songname, user_input_minus_command_name, portMAX_DELAY);

  printf("Sent %s over to the Q_songname\n", user_input_minus_command_name);
  return APP_CLI_STATUS__SUCCESS;
}

void main(void) {
  Q_songname = xQueueCreate(1, sizeof(songname_t));
  Q_songdata = xQueueCreate(1, 512);

  sj2_cli__init();

  xTaskCreate(mp3_reader_task, "reader_task", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(mp3_player_task, "player_task", 4096 / sizeof(void *), NULL, PRIORITY_HIGH, NULL);

  vTaskStartScheduler();
}
#endif

// Lab 08 I2C
#if 1
#if 0
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_slave_functions.h"
#include "i2c_slave_init.h"
#include <stdbool.h>
#include <stdio.h>

static volatile uint8_t slave_memory[256];

bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory) {
  *memory = slave_memory[memory_index];

  return ((slave_memory[memory_index] == *memory) ? true : false);
}

bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value) {
  slave_memory[memory_index] = memory_value;

  return ((memory_value = slave_memory[memory_index]) ? true : false);
}

void turn_on_an_led() { LPC_GPIO2->PIN &= ~(1 << 3); }

void turn_off_an_led() { LPC_GPIO2->PIN |= (1 << 3); }

int main(void) {
  i2c2__slave_init(0x86);

  LPC_IOCON->P2_3 = 0;
  LPC_GPIO2->DIR |= (1 << 3);

  printf("Starting!\n");
  while (1) {
    delay__ms(500);
    if (slave_memory[0] == 0) {
      turn_on_an_led(); // TODO
    } else {
      turn_off_an_led(); // TODO
    }

    return -1;
  }
}
#endif

#if 0
#include "FreeRTOS.h"
#include "cli_handlers.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_slave_functions.h"
#include "i2c_slave_init.h"
#include "sj2_cli.h"
#include "stdio.h"
#include "task.h"
#include <stdbool.h>

static volatile uint8_t slave_memory[256];

bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory) {
  *memory = slave_memory[memory_index];

  printf("\nValue from callback: %i\n", slave_memory[memory_index]);

  return ((slave_memory[memory_index] == *memory) ? true : false);
}

bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value) {
  slave_memory[memory_index] = memory_value;

  return ((memory_value = slave_memory[memory_index]) ? true : false);
}

void test_function(void *p) {
  while (1) {
  }
}

int main(void) {
  sj2_cli__init();

  xTaskCreate(test_function, "test", 4096 / sizeof(void *), NULL, PRIORITY_MEDIUM, NULL);

  vTaskStartScheduler();

  return -1;
}
#endif

/*
// Slave Code
#include "FreeRTOS.h"
#include "gpio.h"
#include "i2c_slave_init.h"
#include "stdint.h"
#include "task.h"

#define SLAVE_ADDRESS 0x20

void main(void) {
  // Set I2C SDA Pin
  gpio__construct_with_function(GPIO__PORT_0, 10, GPIO__FUNCTION_2);
  // Set I2C SCL Pin
  gpio__construct_with_function(GPIO__PORT_0, 11, GPIO__FUNCTION_2);

  i2c2__slave_init(SLAVE_ADDRESS);

  vTaskStartScheduler();
}*/

/*
// Master Code
#include "FreeRTOS.h"
#include "gpio.h"
#include "i2c_slave_init.h"
#include "stdint.h"
#include "task.h"

#define SLAVE_ADDRESS 0x20

void main(void) {
  // Set I2C SDA Pin
  gpio__construct_with_function(GPIO__PORT_0, 10, GPIO__FUNCTION_2);
  // Set I2C SCL Pin
  gpio__construct_with_function(GPIO__PORT_0, 11, GPIO__FUNCTION_2);

  vTaskStartScheduler();
}
*/

#endif

// Lab 07 Watchdogs
#if 0

#include "FreeRTOS.h"
#include "acceleration.h"
#include "cli_handlers.h"
#include "event_groups.h"
#include "ff.h"
#include "queue.h"
#include "sj2_cli.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static QueueHandle_t sensor_queue;
static EventGroupHandle_t xCreatedEventGroup;
static EventBits_t uxBits;
TaskHandle_t task1;
TaskHandle_t task2;
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)

void write_file_using_fatfs_pi(acceleration__axis_data_s *data) {
  const char *filename = "file.txt";
  FIL file; // File handle
  UINT bytes_written = 0;
  FRESULT result = f_open(&file, filename, (FA_WRITE | FA_OPEN_APPEND));

  if (FR_OK == result) {
    char string[64];
    sprintf(string, "%li, %i, %i, %i\n", xTaskGetTickCount(), data->x, data->y, data->z);
    if (FR_OK == f_write(&file, string, strlen(string), &bytes_written)) {
    } else {
      printf("ERROR: Failed to write data to file\n");
    }
    f_close(&file);
  } else {
    printf("ERROR: Failed to open: %s\n", filename);
  }
}

void watchdog_task(void *params) {
  while (1) {

    uxBits = xEventGroupWaitBits(xCreatedEventGroup, BIT_1 | BIT_2, pdTRUE, pdTRUE, 210);
    if ((uxBits & (BIT_1 | BIT_2)) == (BIT_1 | BIT_2)) {
      printf("Verified!\n");
    } else if ((uxBits & BIT_1) == 0 && (uxBits & BIT_2) != 0) {
      printf("Producer Task Failed to Check In!\n");
    } else if ((uxBits & BIT_2) == 0 && (uxBits & BIT_1) != 0) {
      printf("Consumer Task Failed to Check In!\n");
    } else {
      printf("Error with both tasks!\n");
    }
  }
}

void producer_task(void *p) {
  acceleration__axis_data_s temp;
  acceleration__axis_data_s calculated_average;
  while (1) {
    for (int i = 0; i < 100; i++) {
      temp = acceleration__get_data();
      calculated_average.x += temp.x;
      calculated_average.y += temp.y;
      calculated_average.z += temp.z;
      vTaskDelay(1);
    }
    calculated_average.x = calculated_average.x / 100;
    calculated_average.y = calculated_average.y / 100;
    calculated_average.z = calculated_average.z / 100;

    if (xQueueSend(sensor_queue, &calculated_average, 0)) {
      uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_1);
      vTaskDelay(100);
    }
  }
}

void consumer_task(void *p) {
  acceleration__axis_data_s acceleration_value;

  while (1) {
    if (xQueueReceive(sensor_queue, &acceleration_value, portMAX_DELAY)) {
      printf("%li, %i, %i, %i\n", xTaskGetTickCount(), acceleration_value.x, acceleration_value.y,
             acceleration_value.z);
      write_file_using_fatfs_pi(&acceleration_value);

      uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_2);
    }
  }
}

void main(void) {
  sensor_queue = xQueueCreate(1, sizeof(acceleration__axis_data_s));
  xCreatedEventGroup = xEventGroupCreate();

  acceleration__init();
  sj2_cli__init();

  xTaskCreate(producer_task, "Producer", 4096 / sizeof(void *), NULL, PRIORITY_MEDIUM, &task1);
  xTaskCreate(consumer_task, "Consumer", 4096 / sizeof(void *), NULL, PRIORITY_MEDIUM, &task2);
  xTaskCreate(watchdog_task, "Watchdog", 4096 / sizeof(void *), NULL, PRIORITY_HIGH, NULL);

  vTaskStartScheduler();
}

#endif

/*================== Homework Assignment FreeRTOS Producer and Consumer Tasks  ==================*/
/*
Explanation of Observations:
1. Use higher priority for producer task, and note down the order of the print-outs
  - The producer task starts with "Sending..." and then "0 Sent!"
  - The consumer task then starts "Receiving..." and then "0 Received!"
  - The producer is still on Delay of 1000ms, so consumer task starts again with "Receiving..."
  - Nothing is in queue, so consumer sleeps.
  - Producer wakes up and does "Sending..." and "1 Sent!" and goes to sleep for 1000ms
  - Consumer wakes up and continues where it left off, so obtains from queue and prints "1 Received"

2. Use higher priority for consumer task, and note down the order of the print-outs
  - Consumer task starts with "Receiving..."
    - Goes to sleep due to empty queue
  - Producer task sends item to queue and prints "Sending..."
    - Switches context to Consumer because Producer has Lower Precedence
  - Consumer wakes up because there is item in queue and prints " 0 Received!"
  - Consumer still has priority, so reruns task and prints "Receiving..."
  - Consumer goes to sleep due to empty queue
    - Context switches to Producer due to Consumer task sleeping
  - Producer starts where it left off and prints "0 Sent!"
    - Due to Consumer still sleeping, Producer rereuns task
  - Producer sends item to queue and prints "Sending..."
  - Consumer wakes up and runs its task
  - Repeats

3. Use same priority level for both tasks, and note down the order of the print-outs
  - Consumer prints "Receiving..." and sleeps due to empty queue
  - Producer prints "Sending..." and sends item to queue
  - Producer prints "0 Sent!"
  - Consumer then prints "0 Received!" and reruns task
  - Consumer prints "Receiving..." and sleeps due to empty queue
  - Repeats

Additional Questions:
1.  The purpose of the block time during xQueueReceive() is to allow the producer function to send an item into the
queue if the producer function is of lower priority. It essentially allows the CPU to service other tasks while the
current task is waiting for something to appear in the queue.

2.	If we used the value 0 for the block time during xQueueReceive(), then the producer function will never be able
to run if it is of lower priority than the consumer, thus starved. Once it finds out that there is no item in the queue,
it will rerun its task once again in a continuous loop until there is an item in the queue, however, using our first
example from Figure 1, since the producer function is of lower priority, it will never have a chance to send an item to
the queue.

*/
#if 0

#include "FreeRTOS.h"
#include "cli_handlers.h"
#include "gpio.h"
#include "queue.h"
#include "sj2_cli.h"
#include <stdio.h>
#include <stdlib.h>

static QueueHandle_t switch_queue;

typedef enum { switch__off, switch__on } switch_e;

switch_e get_switch_input_from_switch0() {
  switch_e status = switch__off;

  if (LPC_GPIO1->PIN & (1 << 19)) {
    status = switch__on;
  }

  return status;
}

void producer(void *p) {
  while (1) {
    const switch_e switch_value = get_switch_input_from_switch0();

    printf("\nSending...\n");
    if (xQueueSend(switch_queue, &switch_value, 0)) {
      printf("\n%i Sent!\n", switch_value);
    }

    vTaskDelay(1000);
  }
}

void consumer(void *p) {
  switch_e switch_value;
  while (1) {
    printf("\nReceiving...\n");
    if (xQueueReceive(switch_queue, &switch_value, portMAX_DELAY)) {
      printf("\n%i Received!\n", switch_value);
    }
  }
}

void configure_switch() {
  gpio__construct_with_function(GPIO__PORT_1, 19, GPIO__FUNCITON_0_IO_PINgpio);
  LPC_GPIO1->DIR &= ~(1 << 19);
  LPC_IOCON->P1_19 &= ~(3 << 3);
  LPC_IOCON->P1_19 |= (1 << 3);
}

void main(void) {
  configure_switch();
  sj2_cli__init();

  xTaskCreate(producer, "Producer", 4096 / sizeof(void *), NULL, PRIORITY_HIGH, NULL);
  xTaskCreate(consumer, "Consumer", 4096 / sizeof(void *), NULL, PRIORITY_HIGH, NULL);

  // TODO Queue handle is not valid until you create it
  switch_queue =
      xQueueCreate(1, sizeof(switch_e)); // Choose depth of item being our enum (1 should be okay for this example)

  vTaskStartScheduler();
}

#endif

/*================== Lab 06 UART  ==================*/
#if 0
// Lab 6.3
#include "FreeRTOS.h"
#include "clock.h"
#include "gpio.h"
#include "stdlib.h"
#include "task.h"
#include "uart_lab.h"
#include <string.h>

void board_2_receiver_task(void *p) {
  char number_as_string[16] = {0};
  int counter = 0;

  while (true) {
    char byte = 0;
    uart_lab__get_char_from_queue(&byte, portMAX_DELAY);
    printf("Received: %c\n", byte);

    // This is the last char, so print the number
    if ('\0' == byte) {
      number_as_string[counter] = '\0';
      counter = 0;
      printf("Received this number from the other board: %s\n", number_as_string);
    }
    // We have not yet received the NULL '\0' char, so buffer the data
    else {
      number_as_string[counter] = byte;
      counter = counter + 1;
      // TODO: Store data to number_as_string[] array one char at a time
      // Hint: Use counter as an index, and increment it as long as we do not reach max value of 16
    }
  }
}

void uart_io_init(void) {
  // Transmitter
  gpio__construct_with_function(4, 28, GPIO__FUNCTION_2);
  // Receiver
  gpio__construct_with_function(4, 29, GPIO__FUNCTION_2);
}

void main(void) {
  // TODO: Use uart_lab__init() function and initialize UART2 or UART3 (your choice)
  // TODO: Pin Configure IO pins to perform UART2/UART3 function
  uart_io_init();
  uart_lab__init(UART_3, clock__get_core_clock_hz(), 115200);
  uart__enable_receive_interrupt(UART_3);

  xTaskCreate(board_2_receiver_task, "UART_RECEIVER", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);

  vTaskStartScheduler();
}
#endif

#if 0
// Lab 6.3 THIS IS THINGS THAT WE DIDN'T TURN IN, BUT THE ONE ABOVE IS THE ONE I WILL TURN IN
#include "FreeRTOS.h"
#include "clock.h"
#include "gpio.h"
#include "stdlib.h"
#include "task.h"
#include "uart_lab.h"
#include <string.h>

// This task is done for you, but you should understand what this code is doing
/*void board_1_sender_task(void *p) {
  char number_as_string[16] = {0};

  while (true) {
    const int number = rand();
    sprintf(number_as_string, "%i", number);

    // Send one char at a time to the other board including terminating NULL char
    for (int i = 0; i <= strlen(number_as_string); i++) {
      uart_lab__polled_put(UART_3, number_as_string[i]);
      printf("Sent: %c\n", number_as_string[i]);
    }

    printf("Sent: %i over UART to the other board\n", number);
    vTaskDelay(3000);
  }
}*/

void board_2_receiver_task(void *p) {
  char number_as_string[16] = {0};
  int counter = 0;

  while (true) {
    char byte = 0;
    uart_lab__get_char_from_queue(&byte, portMAX_DELAY);
    printf("Received: %c\n", byte);

    // This is the last char, so print the number
    if ('\0' == byte) {
      number_as_string[counter] = '\0';
      counter = 0;
      printf("Received this number from the other board: %s\n", number_as_string);
    }
    // We have not yet received the NULL '\0' char, so buffer the data
    else {
      number_as_string[counter] = byte;
      counter = counter + 1;
      // TODO: Store data to number_as_string[] array one char at a time
      // Hint: Use counter as an index, and increment it as long as we do not reach max value of 16
    }
  }
}

void uart_io_init(void) {
  // Transmitter
  gpio__construct_with_function(4, 28, GPIO__FUNCTION_2);
  // Receiver
  gpio__construct_with_function(4, 29, GPIO__FUNCTION_2);
}

void main(void) {
  // TODO: Use uart_lab__init() function and initialize UART2 or UART3 (your choice)
  // TODO: Pin Configure IO pins to perform UART2/UART3 function
  uart_io_init();
  uart_lab__init(UART_3, clock__get_core_clock_hz(), 115200);
  uart__enable_receive_interrupt(UART_3);

  // xTaskCreate(board_1_sender_task, "UART_SENDER", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(board_2_receiver_task, "UART_RECEIVER", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);

  vTaskStartScheduler();
}

/*
void uart_read_task(void *p) {
  char ch_in;
  while (1) {
    // TODO: Use uart_lab__polled_get() function and printf the received value
    if (uart_lab__get_char_from_queue(&ch_in, portMAX_DELAY)) {
      printf("Received Character: %c", ch_in);
    }
  }
}

void uart_write_task(void *p) {
  while (1) {
    // TODO: Use uart_lab__polled_put() function and send a value
    if (uart_lab__polled_put(UART_3, 'f')) {
      printf("\nSent!\n");
    }
    vTaskDelay(500);
  }
}*/
#endif

/*================== Lab 05 SPI Flash Interface ==================*/
#if 0
#include "FreeRTOS.h"
#include "delay.h"
#include "gpio.h"
#include "semphr.h"
#include "spi.h"
#include "task.h"
#include <stdio.h>

SemaphoreHandle_t spi_bus_mutex;

typedef struct {
  uint8_t manufacturer_id;
  uint8_t device_id_1;
  uint8_t device_id_2;
  uint8_t extended_device_id;
} adesto_flash_id_s;

void adesto_cs(void) {
  LPC_GPIO1->PIN &= ~(1 << 10); // Set *CS LOW
  LPC_GPIO2->PIN &= ~(1 << 0);
}

void adesto_ds(void) {
  LPC_GPIO1->PIN |= (1 << 10); // Set *CS HIGH
  LPC_GPIO2->PIN |= (1 << 0);
}

adesto_flash_id_s adesto_read_signature(void) {
  adesto_flash_id_s data = {0};
  if (xSemaphoreTake(spi_bus_mutex, 1000)) {
    adesto_cs();
    LPC_SSP2->DR = spi__exchange_byte(0x9F); // Sending the Opcode
    data.manufacturer_id = spi__exchange_byte(0xFF);
    data.device_id_1 = spi__exchange_byte(0xFF);
    data.device_id_2 = spi__exchange_byte(0xFF);
    data.extended_device_id = LPC_SSP2->DR;
    printf("\n\n%x, %x, %x, %x\n", data.manufacturer_id, data.device_id_1, data.device_id_2, data.extended_device_id);
    adesto_ds();
    xSemaphoreGive(spi_bus_mutex);
  }
  return data;
}

void spi_pin_configuration(void) {
  // Setting up SCK
  gpio__construct_with_function(GPIO__PORT_1, 0, GPIO__FUNCTION_4);
  // Setting up MOSI
  gpio__construct_with_function(GPIO__PORT_1, 1, GPIO__FUNCTION_4);
  // Setting up MISO
  gpio__construct_with_function(GPIO__PORT_1, 4, GPIO__FUNCTION_4);
  // Setting up *CS
  gpio__construct_with_function(GPIO__PORT_1, 10, GPIO__FUNCITON_0_IO_PIN);
  gpio__construct_as_output(GPIO__PORT_1, 10);

  // Fake *CS
  gpio__construct_with_function(GPIO__PORT_2, 0, GPIO__FUNCITON_0_IO_PIN);
  gpio__construct_as_output(GPIO__PORT_2, 0);
}

void spi_id_verification_task(void *p) {
  while (1) {
    const adesto_flash_id_s id = adesto_read_signature();
    // When we read a manufacturer ID we do not expect, we will kill this task
    if (id.manufacturer_id != 0x1F) {
      fprintf(stderr, "Manufacturer ID read failure\n");
      vTaskSuspend(NULL); // Kill this task
    }
  }
}

void main(void) {
  // TODO: Initialize your SPI, its pins, Adesto flash CS GPIO etc...
  const uint32_t spi_clock_mhz = 24;
  spi__init(spi_clock_mhz);
  spi_pin_configuration();

  spi_bus_mutex = xSemaphoreCreateMutex();

  // Create two tasks that will continously read signature
  xTaskCreate(spi_id_verification_task, "SPI_ID_TASK1", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(spi_id_verification_task, "SPI_ID_TASK2", 4096 / sizeof(void *), NULL, PRIORITY_LOW, NULL);

  vTaskStartScheduler();
}
#endif
#if 0
// Lab 5.1
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
  LPC_GPIO2->PIN &= ~(1 << 0);
}

void adesto_ds(void) {
  LPC_GPIO1->PIN |= (1 << 10); // Set *CS HIGH
  LPC_GPIO2->PIN |= (1 << 0);
}

// TODO: Implement the code to read Adesto flash memory signature
// TODO: Create struct of type 'adesto_flash_id_s' and return it
adesto_flash_id_s adesto_read_signature(void) {
  adesto_flash_id_s data = {0};

  adesto_cs();
  spi__exchange_byte(0x9F); // Sending the Opcode
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

  // Fake *CS
  gpio__construct_with_function(GPIO__PORT_2, 0, GPIO__FUNCITON_0_IO_PIN);
  gpio__construct_as_output(GPIO__PORT_2, 0);
}

void spi_task(void *p) {
  const uint32_t spi_clock_mhz = 1; // Setting clock to 1 MHz to see the waveform generation (as suggested by Preet)
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