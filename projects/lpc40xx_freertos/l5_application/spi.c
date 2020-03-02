#include "spi.h"
#include "lpc40xx.h"
#include <stdint.h>
#include <stdio.h>

#include "clock.h"

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
  LPC_SSP2->CR0 &= ~(0xFF << 8);

  // CONTROL REGISTER 1
  LPC_SSP2->CR1 |= (1 << 1);

  // c) Setup prescalar register to be <= max_clock_mhz
  // {PCLK / [CPSDVSR * (SCR + 1)]}
  LPC_SSP2->CPSR = 8;
}

uint8_t spi__exchange_byte(uint8_t data_out) {
  // Configure the Data register(DR) to send and receive data by checking the status register
  LPC_SSP2->DR = data_out;

  while (LPC_SSP2->SR & (1 << 4)) { // Loop while the SSP controller is busy
    ;
  }

  return (LPC_SSP2->DR);
}