#include "delay.h"
#include "lpc40xx.h"

#define RS (1 << 5)
#define RW (1 << 2)
#define EN (1 << 0)
#define DB7 (1 << 1)
#define DB6 (1 << 4)
#define DB5 (1 << 6)
#define DB4 (1 << 8)

typedef enum lcd_mode {
  COMMAND,
  DATA,
} lcd_mode_Type;

void lcd__write_value(uint8_t value, lcd_mode_Type mode);

void lcd__write_character(char c);

void lcd__clear_display(void);

void set_4_bit_mode(void);

void enable_toggle(void);