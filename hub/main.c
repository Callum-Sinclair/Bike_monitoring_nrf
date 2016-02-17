#include "nrf52.h"
#include "system_nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf.h"                        // Device header
#include "nrf_delay.h"                  // NordicSemiconductor::nRF_Drivers:nrf_delay

#define LED1    17
#define LED2    18
#define LED3    19
#define LED4    20
#define BUT1    13
#define BUT2    14
#define BUT3    15
#define BUT4    16

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

void set_pin(uint8_t pin)
{
    NRF_GPIO->OUTSET = 1UL << pin;
}

void clear_pin(uint8_t pin)
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

int main(void)
{
    gpio_pin_out_init(LED1);
    gpio_pin_out_init(LED2);
    gpio_pin_out_init(LED3);
    gpio_pin_out_init(LED4);
    gpio_pin_in_init(BUT1);
    while (1)
    {
        if (read_pin(BUT1))
        {
            clear_pin(LED1);
            nrf_delay_ms(100);
            clear_pin(LED2);
            nrf_delay_ms(100);
            clear_pin(LED3);
            nrf_delay_ms(100);
            clear_pin(LED4);
            nrf_delay_ms(100);
            set_pin(LED1);
            nrf_delay_ms(100);
            set_pin(LED2);
            nrf_delay_ms(100);
            set_pin(LED3);
            nrf_delay_ms(100);
            set_pin(LED4);
            nrf_delay_ms(100);
        }

    }
}
