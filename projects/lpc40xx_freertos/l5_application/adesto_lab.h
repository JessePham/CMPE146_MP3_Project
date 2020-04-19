#include "clock.h"
#include "lpc40xx.h"
#include "lpc_peripherals.h"
#include "spi.h"
#include <stdint.h>
void adesto_cs(void);
void adesto_ds(void);

// TODO: Study the Adesto flash 'Manufacturer and Device ID' section
typedef struct {
  uint8_t manufacturer_id;
  uint8_t device_id_1;
  uint8_t device_id_2;
  uint8_t extended_device_id;
} adesto_flash_id_s;
