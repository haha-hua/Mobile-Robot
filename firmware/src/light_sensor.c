#include "light_sensor.h"
#include "config.h"
#include "stm32f10x.h"

#define LIGHT_AO_PORT GPIOA
#define LIGHT_AO_PIN GPIO_Pin_4
#define LIGHT_DO_PORT GPIOD
#define LIGHT_DO_PIN GPIO_Pin_11

void light_sensor_init(light_sensor_t *sensor)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    gpio.GPIO_Pin = LIGHT_AO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LIGHT_AO_PORT, &gpio);
    gpio.GPIO_Pin = LIGHT_DO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(LIGHT_DO_PORT, &gpio);

    ADC_DeInit(ADC1);
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) { }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1)) { }

    *sensor = (light_sensor_t){0};
    sensor->do_raw = GPIO_ReadInputDataBit(LIGHT_DO_PORT, LIGHT_DO_PIN) ? 1u : 0u;
    sensor->candidate = sensor->do_raw == LIGHT_DARK_ACTIVE_LEVEL ? 1u : 0u;
    sensor->dark = sensor->candidate;
}

void light_sensor_sample(light_sensor_t *sensor)
{
    uint32_t adc_sum = 0u;
    uint8_t i, raw, dark;
    for (i = 0u; i < LIGHT_ADC_AVG_SAMPLES; i++) {
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) { }
        adc_sum += ADC_GetConversionValue(ADC1);
    }
    sensor->adc_raw = (uint16_t)((adc_sum + LIGHT_ADC_AVG_SAMPLES / 2u) /
                                 LIGHT_ADC_AVG_SAMPLES);
    sensor->millivolts = (uint16_t)(((uint32_t)sensor->adc_raw * LIGHT_ADC_VREF_MV +
                                     LIGHT_ADC_MAX_VALUE / 2u) / LIGHT_ADC_MAX_VALUE);

    raw = GPIO_ReadInputDataBit(LIGHT_DO_PORT, LIGHT_DO_PIN) ? 1u : 0u;
    sensor->do_raw = raw;
    dark = raw == LIGHT_DARK_ACTIVE_LEVEL ? 1u : 0u;
    if (dark != sensor->candidate) {
        sensor->candidate = dark;
        sensor->candidate_samples = 1u;
    } else if (sensor->candidate_samples < LIGHT_STATE_CONFIRM_SAMPLES) {
        sensor->candidate_samples++;
    }
    if (sensor->candidate_samples >= LIGHT_STATE_CONFIRM_SAMPLES)
        sensor->dark = sensor->candidate;
}
