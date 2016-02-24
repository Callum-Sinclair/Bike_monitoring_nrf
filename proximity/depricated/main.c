#include "nrf51.h"
#include "system_nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf.h"                        // Device header
#include "nrf_delay.h"                  // NordicSemiconductor::nRF_Drivers:nrf_delay
#include "gpio_lib.h"
#include "sens_brd.h"

void flash_leds(void)
{
        clear_pin(GREEN_LED);
        nrf_delay_ms(100);
        clear_pin(RED_LED);
        nrf_delay_ms(100);
        clear_pin(BLUE_LED);
        nrf_delay_ms(100);
        set_pin(GREEN_LED);
        nrf_delay_ms(100);
        set_pin(BLUE_LED);
        nrf_delay_ms(100);
        set_pin(RED_LED);
        nrf_delay_ms(100);
}

void init_leds()
{
    gpio_pin_out_init(GREEN_LED);
    gpio_pin_out_init(RED_LED);
    gpio_pin_out_init(BLUE_LED);
}

int main(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    init_leds();
    flash_leds();
    flash_leds();
    volatile uint32_t stat = 0;
    uint32_t i = 0;
    while (1)
    {
        clear_pin(GREEN_LED);        
        //radio_transmit(i);
        clear_pin(BLUE_LED);
        stat = NRF_RADIO->STATE;
        i ++;
        if (i > 4)
        {
            i = 0;
        }
        nrf_delay_ms(1000);
        set_pin(GREEN_LED);
        set_pin(BLUE_LED);
        nrf_delay_ms(1000);
    }
}
