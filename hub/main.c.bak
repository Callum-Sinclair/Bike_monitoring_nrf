#include "nrf52.h"
#include "system_nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf.h"                        // Device header
#include "nrf_delay.h"                  // NordicSemiconductor::nRF_Drivers:nrf_delay
#include "gpio_lib.h"
#include "hub_brd.h"

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
