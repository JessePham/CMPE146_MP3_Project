#include "spi.h"
#include "clock.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include <stdint.h>
#include <stdio.h>

void ssp2_lab__init(uint32_t max_clock_mhz) {
  // Refer to LPC User manual and setup the register bits correctly
  // a) Power on Peripheral
  lpc_peripheral__turn_on_power_to(LPC_PERIPHERAL__SSP2);
  // b) Setup control registers CR0 and CR1
  LPC_SSP2->CR1 = (1 << 1);
  LPC_SSP2->CR0 = (7);
  // c) Setup prescalar register to be <= max_clock_mhz
  uint8_t divider = 4;
  const uint32_t cpu_clock_mhz = clock__get_core_clock_hz() / 1000000UL;
  while (max_clock_mhz < (cpu_clock_mhz / divider) && divider <= 254) {
    divider += 4;
  }

  LPC_SSP2->CPSR = divider;
}

uint8_t ssp2_lab__exchange_byte(uint8_t data_out) {
  // Configure the Data register(DR) to send and receive data by checking the status register
  LPC_SSP2->DR = data_out;
  while (LPC_SSP2->SR & (1 << 4)) {
  }
  return (uint8_t)(LPC_SSP2->DR);
}