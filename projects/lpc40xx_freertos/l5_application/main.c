#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "acceleration.h"
#include "app_cli.h"
#include "board_io.h"
#include "common_macros.h"
#include "event_groups.h"

#include "ff.h"
#include "i2c.h"
#include "lcd.h"
#include "lpc40xx.h"
#include "queue.h"
#include "sj2_cli.h"
#include "sl_string.h"
#include "task.h"
#include <string.h>

void mp3_reader_task(void *p);
void mp3_player_task(void *p);

typedef char songname_t[32];

QueueHandle_t Q_songname;
QueueHandle_t Q_songdata;
FIL file; // File handle

void initialize_lcd_pins(void);
void initialize_lcd_screen(void);

#define RS (1 << 5)
#define RW (1 << 2)
#define EN (1 << 0)
#define DB7 (1 << 1)
#define DB6 (1 << 4)
#define DB5 (1 << 6)
#define DB4 (1 << 8)

int main(void) {
  Q_songname = xQueueCreate(1, sizeof(songname_t));
  Q_songdata = xQueueCreate(1, 512);

  initialize_lcd_pins();
  initialize_lcd_screen();

  xTaskCreate(mp3_reader_task, "mp3_read", 2048 / sizeof(void *), NULL, 2, NULL);
  xTaskCreate(mp3_player_task, "mp3_play", 2048 / sizeof(void *), NULL, 3, NULL);
  sj2_cli__init();
  vTaskStartScheduler();

  return 0;
}
void initialize_lcd_pins(void) {
  // Enable Signal
  gpio__construct_with_function(GPIO__PORT_2, 0, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 0);
  LPC_IOCON->P2_0 &= ~(3 << 3);
  LPC_IOCON->P2_0 |= (1 << 3);
  // Read/Write Signal
  gpio__construct_with_function(GPIO__PORT_2, 2, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 2);
  LPC_IOCON->P2_2 &= ~(3 << 3);
  LPC_IOCON->P2_2 |= (1 << 3);
  // Register Select
  gpio__construct_with_function(GPIO__PORT_2, 5, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 5);
  LPC_IOCON->P2_5 &= ~(3 << 3);
  LPC_IOCON->P2_5 |= (1 << 3);

  // 4-bit Data Bits
  gpio__construct_with_function(GPIO__PORT_2, 1, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 1);
  LPC_IOCON->P2_1 &= ~(3 << 3);
  LPC_IOCON->P2_1 |= (1 << 3);
  gpio__construct_with_function(GPIO__PORT_2, 4, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 4);
  LPC_IOCON->P2_4 &= ~(3 << 3);
  LPC_IOCON->P2_4 |= (1 << 3);
  gpio__construct_with_function(GPIO__PORT_2, 6, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 6);
  LPC_IOCON->P2_6 &= ~(3 << 3);
  LPC_IOCON->P2_6 |= (1 << 3);
  gpio__construct_with_function(GPIO__PORT_2, 8, GPIO__FUNCITON_0_IO_PIN);
  LPC_GPIO2->DIR |= (1 << 8);
  LPC_IOCON->P2_8 &= ~(3 << 3);
  LPC_IOCON->P2_8 |= (1 << 3);
}
void initialize_lcd_screen(void) {
  LPC_GPIO2->CLR |= EN;
  set_4_bit_mode();
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
        lcd__clear_display();
        for (int i = 0; i < strlen(name); ++i) {
          lcd__write_character(name[i]);
        }
        while (!f_eof(&file)) {
          if (FR_OK == f_read(&file, bytes_512, 512, &bytes_read)) {
            printf("\nSong Data Sent: \n");
            xQueueSend(Q_songdata, &bytes_512[0], portMAX_DELAY);
            vTaskDelay(300); // in here just for milestone demo to see bytes on telementary easier
            if (xQueueReceive(Q_songname, &name[0], 0)) {
              break;
            }
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
      printf("\nRecieved Data: \n");
      for (int i = 0; i < (sizeof(bytes_512)); i++) {
        printf("%x", bytes_512[i]);
      }
    }
  }
}

volatile uint8_t slave_memory[256];

bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory) {
  *memory = slave_memory[memory_index];
  if (slave_memory[memory_index] == *memory) {
    return true;
  } else {
    return false;
  }
}

bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value) {
  slave_memory[memory_index] = memory_value;
  if (slave_memory[memory_index] == memory_value) {
    return true;
  } else {
    return false;
  }
}

void i2c2__slave_init(uint8_t slave_address_to_respond_to) {
  // set adress register for slave mode
  LPC_I2C2->ADR0 = (slave_address_to_respond_to << 0);
  // enable for slave functions
  LPC_I2C2->CONSET = 0x44;
}
