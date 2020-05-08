#include "delay.h"
#include "gpio.h"
#include "lpc40xx.h"
#include "stdio.h"
#include "string.h"

#define RS (1 << 5)
#define RW (1 << 2)
#define EN (1 << 0)
#define DB7 (1 << 1)
#define DB6 (1 << 4)
#define DB5 (1 << 6)
#define DB4 (1 << 8)

void initialize_lcd_pins(void);

void initialize_lcd_screen(void);

void lcd__write_value(uint8_t value);

void lcd__write_character(char c);

void lcd__write_name(const char *name);

void lcd__write_continue(const char *value);

void lcd__clear_display(void);

void lcd__set_cursor_position(uint8_t row, uint8_t col);

void lcd__cursor_move_left(void);

void lcd__show_volume(int level);