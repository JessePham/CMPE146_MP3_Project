#include "volume.h"
#include "adc.h"
#include <stdio.h>

uint16_t adc_reading;
int volume_percent;
uint8_t volume_value;
// adc_channel_e volume_channel = ADC__CHANNEL_4;

void init_volume_control(void) {}
uint8_t volume_control(void) {
  adc_reading = adc__get_adc_value(ADC__CHANNEL_4);
  fprintf(stderr, "\nreading: %i\n", adc_reading);
  volume_percent = (100 - ((adc_reading * 100 / 4095)));
  fprintf(stderr, "percent %i\n", volume_percent);
  volume_value = (254 * volume_percent) / 100;
  fprintf(stderr, "volume: %i\n", volume_value);
  return volume_value;
}