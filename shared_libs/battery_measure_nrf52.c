#include "battery_measure_nrf52.h"

static uint32_t adc_sample;

void adc_init(uint8_t a_pin)
{
    // Sets up PIN as an analogue input
    NRF_SAADC->CH[0].PSELP = a_pin + 1;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit;
    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_sample;
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
    uint32_t result;
    NRF_SAADC->EVENTS_DONE = 0;
//    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_sample;
    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_DONE == 0);
    // Max voltage (3V) = 100 % = 255 = 0xFF (8bit)
    // Min voltage (1.2V) = 0 % = 102 = 0x66
    // Range = 153
    result = adc_sample;

    uint8_t bat_percent = (uint8_t)(((adc_sample - BAT_0) * 100) / (BAT_100 - BAT_0)); // calcutate the battery percentage
    
    if (adc_sample < BAT_0)
    {
        return 0;
    }
    return bat_percent;
}
