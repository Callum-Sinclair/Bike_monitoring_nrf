#pragma once
#include "nrf.h"

void adc_init(uint8_t a_pin);
uint8_t battery_measure(uint8_t a_pin);
