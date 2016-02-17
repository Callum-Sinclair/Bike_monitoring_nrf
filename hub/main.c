#include "nrf52.h"
#include "system_nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf.h"                        // Device header
#include "nrf_delay.h"                  // NordicSemiconductor::nRF_Drivers:nrf_delay
#include "gpio_lib.h"
#include "hub_brd.h"

void flash_leds(void)
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

void init_leds()
{
    gpio_pin_out_init(LED1);
    gpio_pin_out_init(LED2);
    gpio_pin_out_init(LED3);
    gpio_pin_out_init(LED4);
}

void enable_radio_rx()
{
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) | \
                       (0 << RADIO_PCNF0_S0LEN_Pos) | \
                       (0 << RADIO_PCNF0_S1LEN_Pos) | \
                       (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
    NRF_RADIO->PCNF1 = (10 << RADIO_PCNF1_MAXLEN_Pos) | \
                       (0 << RADIO_PCNF1_STATLEN_Pos) | \
                       (2 << RADIO_PCNF1_BALEN_Pos) | \
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | \
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

}

uint32_t radio_recieve()
{
    uint32_t data_buff[20];
    NRF_RADIO->PACKETPTR = (uint32_t)&data_buff[0];
    NRF_RADIO->TASKS_START = 1;
    volatile uint32_t state = NRF_RADIO->STATE;
    while (NRF_RADIO->EVENTS_END == 0);
    state = NRF_RADIO->STATE;
    return data_buff[5];
}

int main(void)
{
    init_leds();
    gpio_pin_in_init(BUT1);
    flash_leds();
    flash_leds();
    
    uint32_t recieved_val;
    
    enable_radio_rx();
    while (1)
    {
        recieved_val = radio_recieve();
        set_pin(LED1);
        set_pin(LED2);
        set_pin(LED3);
        set_pin(LED4);
        switch (recieved_val)
        {
            case 1: clear_pin(LED1);
            break;
            case 2: clear_pin(LED2);
            break;
            case 3: clear_pin(LED3);
            break;
            case 4: clear_pin(LED4);
            break;
            default:
                    clear_pin(LED1);
                    clear_pin(LED2);
                    clear_pin(LED3);
                    clear_pin(LED4);
            break;
        }
    }
}
