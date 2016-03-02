#pragma once
#include "nrf.h"
#define BAT_100 255
#define BAT_0   102

void adc_init(uint8_t a_pin);
uint8_t battery_measure(uint8_t a_pin);
