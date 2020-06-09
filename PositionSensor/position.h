#ifndef __POSITION_H
#define __POSITION_H

#include "sys.h"


typedef struct
{
    int raw_angle;
    int oldAngle;
    float position;
    float oldPosition;

    float MechPosition;
    float MechPositionFixed;

    float MechOffset;
    float ElecPosition;
    float ElecOffset;

    int rotations;

    float MechVelocity;
    float ElecVelocity;
    float ElecVeloFilt;
    float velVec[40];

    int offset_lut[128];  //linear error table

} PositionSensor;

extern PositionSensor PS;
void PS_Value(float dt); //calculate position & velocity
void LUT_Write(int new_lut[128]);


#endif
