#pragma once
#include "nrf.h"
#define BAT_100 213
#define BAT_0   99

void bat_adc_init(uint8_t a_pin);
uint8_t battery_measure(uint8_t a_pin);
