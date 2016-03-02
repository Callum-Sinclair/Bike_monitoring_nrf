#include "gpio_lib.h"

void gpio_pin_in_init(uint8_t pin)
{
    NRF_GPIO->PIN_CNF[pin] = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos \
                           | GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos \
                           | GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos \
                           | GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos \
                           | GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos;
}

void gpio_pin_out_init(uint8_t pin)
{
    NRF_GPIO->PIN_CNF[pin] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos \
                           | GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos \
                           | GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos;
}

volatile void set_pin(uint8_t pin)
{
    NRF_GPIO->OUTSET = 1UL << pin;
}

volatile void clear_pin(uint8_t pin)
{
    NRF_GPIO->OUTCLR = 1UL << pin;
}

uint8_t read_pin(uint8_t pin)
{
    uint32_t pin_in = NRF_GPIO->IN;
    if ((pin_in) & (1UL << pin))
    {
        return 1;
    }
    return 0;
}
