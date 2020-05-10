#include "lcd_lab.h"

static void lcd__clock_pulse(void) {
  LPC_GPIO2->SET |= EN;
  LPC_GPIO2->CLR |= EN;
}

static void lcd__set_databits(uint8_t value) {
  if ((value & (1 << 0)))
    LPC_GPIO2->SET |= DB4;
  else
    LPC_GPIO2->CLR |= DB4;
  if ((value & (1 << 1)))
    LPC_GPIO2->SET |= DB5;
  else
    LPC_GPIO2->CLR |= DB5;
  if ((value & (1 << 2)))
    LPC_GPIO2->SET |= DB6;
  else
    LPC_GPIO2->CLR |= DB6;
  if ((value & (1 << 3)))
    LPC_GPIO2->SET |= DB7;
  else
    LPC_GPIO2->CLR |= DB7;
}

void lcd__write_value(uint8_t value) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->SET |= RS;
  LPC_GPIO2->CLR |= RW;

  lcd__set_databits(value);
  lcd__clock_pulse();
  delay__ms(1);
}

void lcd__write_character(char c) {
  delay__ms(1);
  lcd__write_value((int)c >> 4);
  lcd__write_value((int)c);
}

void lcd__clear_display(void) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;

  lcd__set_databits(0x00);
  lcd__clock_pulse();

  delay__ms(1);

  lcd__set_databits(0x01);
  lcd__clock_pulse();

  delay__ms(1);
}

void lcd__write_name(const char *name) {
  lcd__clear_display();
  for (int i = 0; i < strlen(name); i++) {
    lcd__write_character(name[i]);
  }
}

void lcd__write_continue(const char *value) {
  for (int i = 0; i < strlen(value); i++) {
    lcd__write_character(value[i]);
  }
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
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;

  // Send 0011
  delay__ms(15);
  lcd__set_databits(0x03);
  lcd__clock_pulse();

  // Send 0011
  delay__ms(15);
  lcd__set_databits(0x03);
  lcd__clock_pulse();

  // Send 0011
  delay__ms(15);
  lcd__set_databits(0x03);
  lcd__clock_pulse();

  // Send 0010
  delay__ms(15);
  lcd__set_databits(0x02);
  lcd__clock_pulse();

  // Send 0010
  delay__ms(15);
  lcd__set_databits(0x02);
  lcd__clock_pulse();

  // Send 1100
  delay__ms(15);
  lcd__set_databits(0x0C);
  lcd__clock_pulse();

  // Send 0000
  delay__ms(15);
  lcd__set_databits(0x00);
  lcd__clock_pulse();

  // Send 1111
  delay__ms(15);
  lcd__set_databits(0x0F);
  lcd__clock_pulse();

  // Send 0000
  delay__ms(15);
  lcd__set_databits(0x00);
  lcd__clock_pulse();

  // Send 0001
  delay__ms(15);
  lcd__set_databits(0x01);
  lcd__clock_pulse();

  // Send 0000
  delay__ms(15);
  lcd__set_databits(0x00);
  lcd__clock_pulse();

  // Send 0110
  delay__ms(15);
  lcd__set_databits(0x06);
  lcd__clock_pulse();
}

void lcd__set_cursor_position(uint8_t row, uint8_t col) {
  uint8_t position = (col + (row * 40)) + 0x80;
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;

  lcd__set_databits(position >> 4);
  lcd__clock_pulse();
  lcd__set_databits(position);
  lcd__clock_pulse();

  delay__ms(1);
}

void lcd__cursor_move_left(void) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;

  lcd__set_databits(0x00);
  lcd__clock_pulse();
  lcd__set_databits(0x0C);
  lcd__clock_pulse();

  delay__ms(1);
}

void lcd__cursor_move_right(void) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;

  lcd__set_databits(0x00);
  lcd__clock_pulse();
  lcd__set_databits(0x08);
  lcd__clock_pulse();

  delay__ms(1);
}

void lcd__show_levels(uint8_t volume_level, uint8_t treble_level, uint8_t bass_level) {
  uint8_t max_index = 4;
  lcd__set_cursor_position(1, 0);
  lcd__write_continue("V:");

  for (int i = 0; i < volume_level / 2; i++) {
    lcd__write_character(0xFF);
    max_index--;
  }

  for (max_index; max_index > 0; max_index--) {
    lcd__write_character(0x10);
  }

  lcd__write_character(0x10);
  lcd__write_continue("T:");
  lcd__write_digit(treble_level);

  lcd__write_character(0x10);
  lcd__write_continue("B:");
  lcd__write_digit(bass_level);
}

void lcd__write_digit(uint8_t digit) {
  switch (digit) {
  case 0:
    lcd__write_character(0x30);
    break;
  case 1:
    lcd__write_character(0x31);
    break;
  case 2:
    lcd__write_character(0x32);
    break;
  case 3:
    lcd__write_character(0x33);
    break;
  case 4:
    lcd__write_character(0x34);
    break;
  case 5:
    lcd__write_character(0x35);
    break;
  case 6:
    lcd__write_character(0x36);
    break;
  case 7:
    lcd__write_character(0x37);
    break;
  case 8:
    lcd__write_character(0x38);
    break;
  case 9:
    lcd__write_character(0x39);
    break;
  default:
    lcd__write_character(' ');
    break;
  }
}