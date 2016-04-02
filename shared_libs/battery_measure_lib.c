#include "battery_measure_lib.h"

void bat_adc_init(uint8_t a_pin)
{
    // Sets up PIN as an analogue input
    NRF_ADC->CONFIG = ((ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) | \
                       (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) | \
                       (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) | \
                       ((1UL << a_pin) << ADC_CONFIG_PSEL_Pos) | \
                       (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos));
    
    
    NRF_ADC->ENABLE = 1;
}

uint8_t battery_measure(uint8_t a_pin)
{
    uint32_t result;
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->TASKS_START = 1;
    while (NRF_ADC->EVENTS_END == 0);
    // Max voltage (3 V) = 100 % = 213 = 0xD5 (8bit)
    // Min voltage (1.4 V) = 0 % = 99  = 0x63
    result = NRF_ADC->RESULT;

    uint8_t bat_percent = (uint8_t)(((result - BAT_0) * 100) / (BAT_100 - BAT_0)); // calcutate the battery percentage
    
    /*if (result < BAT_0)
    {
        return 1;
    }*/
    return result;
    return bat_percent;
}
