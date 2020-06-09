#ifndef __tim_H
#define __tim_H

#include "sys.h"

#define TIM1_ARR    2016-1    //168*24/2-1
#define DEADTIME   0 //DTG[7:0]  ����ʹ�õ�MOS��TPH2R506PL�ֲ�P3���ر��½�ʱ����39ns //tdts = tck_int = 1/168M ԼΪ6ns����dtg = 7���͹���

#define MOTOR_BREAK    {TIM1->EGR &= 0xFFFF; TIM1->EGR |= 0x80;}   //TIM1_EGR_BG = 1; ���ɶ�·�����¼�
#define MOTOR_DISBREAK {TIM1->SR &= 0xFF7F; TIM1->BDTR |= 0x8000;} //TIM1_EGR_BG = 0; �����·�����¼��������·�����жϱ�־λ��ʹ��MOE���

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);


#endif

