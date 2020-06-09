#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#define R_PHASE 0.105f          //Ohms
#define L_D 0.00003f            //Henries
#define L_Q 0.00003f            //Henries
#define KT .075f                //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 20                  //Number of pole pairs
#define GR 1.0f                 //Gear ratio
#define KT_OUT 0.45f            //KT*GR
#define WB 0.0024f              //Webers.  
#define R_TH 1.25f              //Kelvin per watt
#define INV_M_TH 0.03125f       //Kelvin per joule


#endif
