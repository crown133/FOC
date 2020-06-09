#ifndef __LED_H_
#define __LED_H_

#include "sys.h"

#define led_on   PAout(2) = 0;
#define led_off  PAout(2) = 1;


void led_Init(void);


#endif 

