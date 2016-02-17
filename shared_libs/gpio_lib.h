#pragma once
#include "nrf.h"

void gpio_pin_in_init(uint8_t pin);
void gpio_pin_out_init(uint8_t pin);
volatile void set_pin(uint8_t pin);
volatile void clear_pin(uint8_t pin);
uint8_t read_pin(uint8_t pin);
