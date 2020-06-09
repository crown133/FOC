#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H


#define I_SCALE 0.02014160156f  // Amps per A/D Count / formulation in datasheet p64 / (3.3/0.04/4096)
#define V_SCALE 0.00786f      //0.008096        // // Bus volts per A/D Count

#define DTC_MAX 0.94f           // Max phase duty cycle  //为保证电阻采样，一般PWM最大占空比限制�?95%
#define DTC_MIN 0.0f           // Min phase duty cycle

#define PWM_ARR 2099         // 2016 - 1 timer autoreload value


#endif
