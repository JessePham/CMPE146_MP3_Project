#include "lcd.h"
#include "stdio.h"

void lcd__write_value(uint8_t value, lcd_mode_Type mode) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RW;
  if (mode == DATA) {
    LPC_GPIO2->SET |= RS;
    printf("DATA: %x\n", value);
  }
  if (mode == COMMAND) {
    LPC_GPIO2->CLR |= RS;
    printf("COMMAND: %x\n", value);
  }
  LPC_GPIO2->CLR |= RW;

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

  enable_toggle();
  delay__ms(1);
}

void lcd__write_character(char c) {
  char first_val = c & 0xF;
  char second_val = c >> 4;
  printf("Value: 0x%x\n", (int)c);
  lcd__write_value((int)second_val, DATA);
  printf("High Bits: 0x%x\n", (int)second_val);
  delay__ms(1);
  lcd__write_value((int)first_val, DATA);
  printf("Low Bits: 0x%x\n\n", (int)first_val);
  delay__ms(1);
}

void lcd__clear_display(void) {
  LPC_GPIO2->CLR |= EN;
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->CLR |= (DB4 | DB5 | DB6 | DB7);
  enable_toggle();
  delay__ms(1);

  LPC_GPIO2->CLR |= (DB5 | DB6 | DB7);
  LPC_GPIO2->SET |= DB4;
  enable_toggle();
  delay__ms(1);
}

void enable_toggle(void) {
  LPC_GPIO2->SET |= EN;
  LPC_GPIO2->CLR |= EN;
  delay__ms(1);
}

void set_4_bit_mode(void) {
  // Send 0011
  delay__ms(15);
  lcd__write_value(0b0011, COMMAND);

  enable_toggle();

  // Send 0011
  delay__ms(15);
  lcd__write_value(0b0011, COMMAND);

  enable_toggle();

  // Send 0011
  delay__ms(15);
  lcd__write_value(0b0011, COMMAND);

  enable_toggle();

  delay__ms(15);
  // Send 0010
  lcd__write_value(0b0010, COMMAND);

  enable_toggle();

  delay__ms(15);
  // Send 0010
  lcd__write_value(0b0010, COMMAND);

  enable_toggle();
  delay__ms(15);

  // Send 1100
  lcd__write_value(0b1100, COMMAND);

  enable_toggle();

  delay__ms(15);
  // Send 0000
  lcd__write_value(0b0000, COMMAND);

  enable_toggle();

  delay__ms(15);
  // Send 1111
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->SET |= DB7;
  LPC_GPIO2->SET |= DB6;
  LPC_GPIO2->SET |= DB5;
  LPC_GPIO2->SET |= DB4;
  enable_toggle();

  delay__ms(15);
  // Send 0000
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->CLR |= DB7;
  LPC_GPIO2->CLR |= DB6;
  LPC_GPIO2->CLR |= DB5;
  LPC_GPIO2->CLR |= DB4;

  enable_toggle();

  delay__ms(15);
  // Send 0001
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->CLR |= DB7;
  LPC_GPIO2->CLR |= DB6;
  LPC_GPIO2->CLR |= DB5;
  LPC_GPIO2->SET |= DB4;

  enable_toggle();

  delay__ms(15);
  // Send 0000
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->CLR |= DB7;
  LPC_GPIO2->CLR |= DB6;
  LPC_GPIO2->CLR |= DB5;
  LPC_GPIO2->CLR |= DB4;

  enable_toggle();

  delay__ms(15);
  // Send 0110
  LPC_GPIO2->CLR |= RS;
  LPC_GPIO2->CLR |= RW;
  LPC_GPIO2->CLR |= DB7;
  LPC_GPIO2->SET |= DB6;
  LPC_GPIO2->SET |= DB5;
  LPC_GPIO2->CLR |= DB4;

  enable_toggle();
}
