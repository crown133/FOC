#ifndef __tim_H
#define __tim_H

#include "sys.h"

#define TIM1_ARR    2016-1    //168*24/2-1
#define DEADTIME   0 //DTG[7:0]  根据使用的MOS管TPH2R506PL手册P3，关闭下降时间在39ns //tdts = tck_int = 1/168M 约为6ns，故dtg = 7差不多就够了

#define MOTOR_BREAK    {TIM1->EGR &= 0xFFFF; TIM1->EGR |= 0x80;}   //TIM1_EGR_BG = 1; 生成断路输入事件
#define MOTOR_DISBREAK {TIM1->SR &= 0xFF7F; TIM1->BDTR |= 0x8000;} //TIM1_EGR_BG = 0; 清除断路生成事件，清除断路输入中断标志位，使能MOE输出

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);


#endif

