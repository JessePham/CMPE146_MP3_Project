#include "ff.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "cli_handlers.h"
#include "lpc_peripherals.h"

#include "adc.h"
#include "delay.h"
#include "gpio.h"
#include "lcd.h"
#include "queue.h"
#include "sj2_cli.h"
#include "song_list.h"
#include "spi.h"
#include "ssp2.h"
#include "string.h"
#include "volume.h"

void mp3_reader_task(void *p);
void mp3_player_task(void *p);
void next_song_task(void *p);
void prev_song_task(void *p);

void display_songs_on_lcd(void *p);
void initialize_buttons(void);
void initialize_SPI_GPIO(void);

void initialize_decoder(void);
void SCI_WRITE(uint8_t address, uint16_t value);
void SDI_WRITE(uint8_t *data_buffer, uint8_t count);
void set_volume(uint8_t volume);
uint16_t SCI_READ(uint8_t address);
bool mp3_decoder_needs_data(void);
void change_volume_task(void *p);

typedef char songname_t[32];

QueueHandle_t Q_songname;
QueueHandle_t Q_songdata;
SemaphoreHandle_t next_song_signal;
SemaphoreHandle_t prev_song_signal;

gpio_s next_button = {0, 30};
gpio_s prev_button = {0, 29};

gpio_s SCK = {1, 0};
gpio_s MOSI = {1, 1};
gpio_s MISO = {1, 4};
gpio_s XDCS = {0, 15};
gpio_s MP3CS = {1, 31};
gpio_s DREQ = {1, 20};
gpio_s RESET = {0, 9};
gpio_s volume_knob = {1, 30};

FIL file;

void set_volume(uint8_t volume) {
  uint16_t _volume;
  _volume = (volume << 8) + volume;

  SCI_WRITE(0x0B, _volume);
}

int main(void) {
  Q_songname = xQueueCreate(1, sizeof(songname_t));
  Q_songdata = xQueueCreate(1, 512);
  next_song_signal = xSemaphoreCreateBinary();
  prev_song_signal = xSemaphoreCreateBinary();

  initialize_lcd_pins();
  initialize_lcd_screen();
  initialize_buttons();
  song_list__populate();

  gpio__set_as_input(volume_knob);
  gpio__construct_with_function(1, 30, 3);

  ssp2__initialize(24000);
  initialize_SPI_GPIO();
  initialize_decoder();

  xTaskCreate(change_volume_task, "change_volume", 2048 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(next_song_task, "next_song", 2048 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(prev_song_task, "prev_song", 2048 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(display_songs_on_lcd, "display_song", 2048 / sizeof(void *), NULL, PRIORITY_LOW, NULL);
  xTaskCreate(mp3_reader_task, "mp3_read", 2048 / sizeof(void *), NULL, PRIORITY_MEDIUM, NULL);
  xTaskCreate(mp3_player_task, "mp3_play", 2048 / sizeof(void *), NULL, PRIORITY_HIGH, NULL);
  sj2_cli__init();
  vTaskStartScheduler();
  return 0;
}
void change_volume_task(void *p) {
  uint8_t volume;
  adc__initialize();
  LPC_IOCON->P1_30 |= (1 << 3);
  LPC_IOCON->P1_30 &= ~(1 << 7);
  while (1) {
    volume = volume_control();
    printf("%x\n", volume);
    set_volume(volume);
    vTaskDelay(1);
  }
}
uint16_t SCI_READ(uint8_t address) {
  while (!mp3_decoder_needs_data())
    ;
  gpio__reset(MP3CS);
  ssp2__exchange_byte(0x03);
  ssp2__exchange_byte(address);
  uint8_t response1 = ssp2__exchange_byte(0xFF);
  while (!mp3_decoder_needs_data())
    ;
  uint8_t response2 = ssp2__exchange_byte(0xFF);
  while (!mp3_decoder_needs_data())
    ;
  gpio__set(MP3CS);
  uint16_t resultValue = response1 << 8;
  resultValue |= response2;
  return resultValue;
}

void initialize_decoder(void) {
  gpio__reset(MP3CS);
  gpio__reset(RESET);
  delay__ms(10);
  gpio__set(RESET);
  delay__ms(10);
  SCI_WRITE(0x00, 0xC890);

  ssp2__set_max_clock(1000);
  ssp2__exchange_byte(0xFF);
  gpio__set(XDCS);
  delay__ms(10);

  set_volume(40);

  uint16_t SCI_CLOCK = SCI_READ(0x03);

  printf("start_up clock: 0x%x\n", SCI_CLOCK);

  SCI_WRITE(0x03, 0x6000); // set multiplier to 3x

  printf("initialized clock: 0x%x\n", SCI_CLOCK);
}
void SDI_WRITE(uint8_t *data_buffer, uint8_t count) {
  gpio__reset(XDCS);

  while (count--) {
    ssp2__exchange_byte(*data_buffer);
    data_buffer++;
  }

  gpio__set(XDCS);
}

void SCI_WRITE(uint8_t address, uint16_t value) {
  while (gpio__get(DREQ) == 0)
    ;
  gpio__reset(MP3CS);

  ssp2__exchange_byte(0x02);
  ssp2__exchange_byte(address);
  ssp2__exchange_byte(value >> 8);
  ssp2__exchange_byte(value & 0xFF);
  gpio__set(MP3CS);
  while (gpio__get(DREQ) == 0)
    ;
}

void next_song_task(void *p) {
  while (1) {
    if (gpio__get(next_button)) {
      xSemaphoreGive(next_song_signal);
      vTaskDelay(500);
    }
  }
}
void prev_song_task(void *p) {
  while (1) {
    if (gpio__get(prev_button)) {
      xSemaphoreGive(prev_song_signal);
      vTaskDelay(500);
    }
  }
}

void initialize_buttons() {
  gpio__set_function(next_button, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_input(next_button);
  gpio__set_function(prev_button, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_input(prev_button);
}

void initialize_SPI_GPIO() {
  gpio__construct_with_function(GPIO__PORT_1, 0, GPIO__FUNCTION_4); // SCK
  gpio__construct_with_function(GPIO__PORT_1, 1, GPIO__FUNCTION_4); // MOSI
  gpio__construct_with_function(GPIO__PORT_1, 4, GPIO__FUNCTION_4); // MISO

  // CS is ACTIVE LOW!!!
  gpio__set_function(XDCS, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_output(XDCS);
  gpio__set_function(MP3CS, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_output(MP3CS);

  gpio__set_function(DREQ, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_input(DREQ);
  gpio__set_function(RESET, GPIO__FUNCITON_0_IO_PIN);
  gpio__set_as_output(RESET);
}

void display_songs_on_lcd(void *p) {
  int song_index = 0;
  const char *current_song = song_list__get_name_for_item(song_index);
  lcd__clear_display();
  lcd__write_name(current_song);

  while (1) {
    if (xSemaphoreTake(next_song_signal, 100)) {
      if (++song_index >= song_list__get_item_count()) {
        song_index = 0;
      }
      current_song = song_list__get_name_for_item(song_index);
      lcd__clear_display();
      lcd__write_name(current_song);
      vTaskDelay(500);
    }

    if (xSemaphoreTake(prev_song_signal, 100)) {
      if (--song_index < 0) {
        song_index = song_list__get_item_count() - 1;
      }
      current_song = song_list__get_name_for_item(song_index);
      lcd__clear_display();
      lcd__write_name(current_song);
      vTaskDelay(500);
    }
  }
}
void mp3_reader_task(void *p) {
  songname_t name;
  unsigned char bytes_512[512];
  UINT bytes_read = 0;
  while (1) {
    if (xQueueReceive(Q_songname, &name[0], portMAX_DELAY)) {
      FRESULT read_file = f_open(&file, &name[0], FA_READ);
      if (FR_OK == read_file) {
        while (!f_eof(&file)) {
          if (FR_OK == f_read(&file, bytes_512, 512, &bytes_read)) {
            xQueueSend(Q_songdata, &bytes_512[0], portMAX_DELAY);
            vTaskDelay(2);
          } else {
            printf("ERRO  R: Failed to read data to file\n");
          }
        }
        printf("\nDone\n");
      } else {
        printf("file doesnt exist\n");
      }
      f_close(&file);
    }
  }
}

bool mp3_decoder_needs_data(void) { return gpio__get(DREQ); }

void mp3_player_task(void *p) {
  unsigned char bytes_512[512];
  while (1) {
    if (xQueueReceive(Q_songdata, &bytes_512[0], portMAX_DELAY)) {
      size_t byte_counter = 0;
      while (byte_counter < sizeof(bytes_512)) {
        if (mp3_decoder_needs_data()) {
          gpio__reset(XDCS);
          for (size_t byte = byte_counter; byte < (byte_counter + 32); byte++) {
            ssp2__exchange_byte(bytes_512[byte]);
            // printf("%x ", bytes_512[byte]);
          }
          gpio__set(XDCS);
          byte_counter += 32;
        } else {
          vTaskDelay(1);
        }
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
