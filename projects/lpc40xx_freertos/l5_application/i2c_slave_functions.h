#include "stdint.h"
#include <stdbool.h>

static volatile uint8_t slave_memory[256];

/**
 * Use memory_index and read the data to *memory pointer
 * return true if everything is well
 */
bool i2c_slave_callback__read_memory(uint8_t memory_index, uint8_t *memory) {
  *memory = slave_memory[memory_index];

  return ((slave_memory[memory_index] == *memory) ? true : false);
};

/**
 * Use memory_index to write memory_value
 * return true if this write operation was valid
 */
bool i2c_slave_callback__write_memory(uint8_t memory_index, uint8_t memory_value) {
  slave_memory[memory_index] = memory_value;

  return ((memory_value = slave_memory[memory_index]) ? true : false);
};

// TODO: You can write the implementation of these functions in your main.c (i2c_slave_functionc.c is optional)