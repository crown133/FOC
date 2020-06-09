#ifndef __ADC_H
#define __ADC_H

#include "sys.h"
#include "structs.h"


#define CH_NUM         3
#define ADC_SAMPLE_NUM 5

#define ADC_BUS_CHANNEL	    ADC_CHANNEL_10
#define ADC_PHASE1_CHANNEL  ADC_CHANNEL_12
#define ADC_PHASE2_CHANNEL  ADC_CHANNEL_13

extern uint32_t adc_buf[CH_NUM * ADC_SAMPLE_NUM];
extern uint32_t adc_val[3];

// void MX_ADC1_Init(void);
// void ADC_Filter(ControllerStruct* controller);
void ADC_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

#endif

