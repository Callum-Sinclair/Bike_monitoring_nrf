#include "battery_measure_nrf52.h"

void adc_init(uint8_t a_pin)
{
    // Sets up PIN as an analogue input
    NRF_SAADC->CH[0].PSELP = 1UL << a_pin;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit;
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos) | \
                               (SAADC_CH_CONFIG_RESN_Bypass   << SAADC_CH_CONFIG_RESN_Pos) | \
                               (SAADC_CH_CONFIG_GAIN_Gain1_4  << SAADC_CH_CONFIG_GAIN_Pos) | \
                               (SAADC_CH_CONFIG_REFSEL_VDD1_4 << SAADC_CH_CONFIG_REFSEL_Pos) | \
                               (SAADC_CH_CONFIG_TACQ_10us     << SAADC_CH_CONFIG_TACQ_Pos) | \
                               (SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos));

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
    //uint32_t result = adc_sample;
    
    return (uint8_t)result;
}