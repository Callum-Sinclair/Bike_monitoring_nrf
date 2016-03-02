#include "battery_measure_nrf52.h"

void adc_init(uint8_t a_pin)
{
    // Sets up PIN as an analogue input
    NRF_SAADC->CH[0].PSELP = 1UL << a_pin;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit;
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) | \
                               (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) | \
                               (SAADC_CH_CONFIG_GAIN_Gain1_5    << SAADC_CH_CONFIG_GAIN_Pos) | \
                               (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) | \
                               (SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos) | \
                               (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)); //gives Vref 0.6 V, but input gain 1/5, so 100% input = 3V

    NRF_SAADC->ENABLE = 1;    
}

uint8_t battery_measure(uint8_t a_pin)
{
    uint32_t adc_sample;
    
    NRF_SAADC->EVENTS_DONE = 0;
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_sample;
    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_DONE == 0);
    // Max voltage (3V) = 100 % = 255 = 0xFF (8bit)
    // Min voltage (1.2V) = 0 % = 102 = 0x66
    // Range = 153
    
    uint8_t bat_percent = (uint8_t)(((adc_sample - BAT_0) * 100) / (BAT_100 - BAT_0)); // calcutate the battery percentage
    
    return bat_percent;
}
