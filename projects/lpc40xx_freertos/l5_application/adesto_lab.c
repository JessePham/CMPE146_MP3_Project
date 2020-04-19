#include "adesto_lab.h"
#include "clock.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include "spi.h"
#include <stdint.h>
#include <stdio.h>

void adesto_cs(void) {
  LPC_GPIO1->PIN &= ~(1 << 10); // active low chip select
  LPC_GPIO2->PIN &= ~(1 << 1);
}

void adesto_ds(void) {
  LPC_GPIO1->PIN |= (1 << 10);
  LPC_GPIO1->PIN |= (1 << 10);
  LPC_GPIO2->PIN |= (1 << 1);
}
