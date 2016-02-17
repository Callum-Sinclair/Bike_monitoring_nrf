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

void enable_radio_tx()
{
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) | \
                       (0 << RADIO_PCNF0_S0LEN_Pos) | \
                       (0 << RADIO_PCNF0_S1LEN_Pos);
    NRF_RADIO->PCNF1 = (10 << RADIO_PCNF1_MAXLEN_Pos) | \
                       (0 << RADIO_PCNF1_STATLEN_Pos) | \
                       (2 << RADIO_PCNF1_BALEN_Pos) | \
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | \
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);
}

void radio_transmit(uint32_t val)
{
    uint32_t data_buff[20];
    for (uint8_t i = 0; i < 20; i++)
    {
        data_buff[i] = val;
    }
    NRF_RADIO->PACKETPTR = (uint32_t)&data_buff[0];
    NRF_RADIO->TASKS_START = 1;
    volatile uint32_t state = NRF_RADIO->STATE;
    while (NRF_RADIO->EVENTS_END == 0);
}

int main(void)
{
    init_leds();
    flash_leds();
    flash_leds();
    volatile uint32_t stat = 0;
    enable_radio_tx();
    uint32_t i = 0;
    while (1)
    {
        clear_pin(GREEN_LED);        
        radio_transmit(i);
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
