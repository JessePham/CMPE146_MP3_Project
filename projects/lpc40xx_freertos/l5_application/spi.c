#include "spi.h"
#include "lpc40xx.h"
#include <stdint.h>
#include <stdio.h>

void spi__init(uint32_t max_clock_mhz) {
  // Refer to LPC User manual and setup the register bits correctly
  // a) Power on Peripheral
  const uint32_t spi_enable = (1 << 20);
  LPC_SC->PCONP |= spi_enable;
  // b) Setup control registers CR0 and CR1 (Pg. 611)

  // CONTROL REGISTER 0
  // Setting as 8-bit transfer bits (3:0)
  LPC_SSP2->CR0 &= ~(0xF);
  LPC_SSP2->CR0 |= (7 << 0);
  LPC_SSP2->CR0 &= ~(3 << 4);
  LPC_SSP2->CR0 &= ~(0xFF << 8); // SCR = 0

  // CONTROL REGISTER 1
  LPC_SSP2->CR1 |= (1 << 1); // Enables SSP Controller

  // c) Setup prescalar register to be <= max_clock_mhz
  // {PCLK / [CPSDVSR * (SCR + 1)]}

  uint8_t dvsr = 2;
  uint32_t cpu_clock_MHz = 96;
  // dividing the cpu clock's cycle until it is less than the input clock
  while (max_clock_mhz < (cpu_clock_MHz / dvsr) && dvsr <= 254) {
    dvsr += 2; // divider is in increments of 2 as per stated in page 613: "This even value between 2 and 254..."
  }

  LPC_SSP2->CPSR = dvsr;
}

uint8_t spi__exchange_byte(uint8_t data_out) {
  // Configure the Data register(DR) to send and receive data by checking the status register
  LPC_SSP2->DR = data_out;

  while (LPC_SSP2->SR & (1 << 4)) { // Loop while the SSP controller is busy
    ;
  }

  return (LPC_SSP2->DR);
}