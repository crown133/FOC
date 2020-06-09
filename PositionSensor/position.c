#include "position.h"
#include "FastMath.h"
#include "motor_config.h"

#include "TLE5012B.h"
#include "string.h"

PositionSensor PS;

void PS_Value(float dt) //calculate position & velocity
{
    PS.raw_angle = (float)(TLE5012_ReadValue(READ_ANGLE_VALUE) & 0x7FFF);

    int off_1 = PS.offset_lut[PS.raw_angle>>8];
    int off_2 = PS.offset_lut[((PS.raw_angle>>8)+1)%128];
    int off_interp = off_1 + ((off_2 - off_1)*(PS.raw_angle - ((PS.raw_angle>>8)<<8))>>8);        // Interpolate between lookup table entries
    int angle = PS.raw_angle + off_interp;        // Correct for nonlinearity with lookup table from calibration
    if (angle - PS.oldAngle > _CPR / 2)
    {
        PS.rotations -= 1;
    }
    else if (angle - PS.oldAngle < -_CPR / 2)
    {
        PS.rotations += 1;
    }
    PS.oldAngle = angle;

    PS.oldPosition = PS.position;
    PS.position = 2.0f * PI * angle / (float)_CPR;

    PS.MechPosition = 2.0f * PI * (angle / (float)_CPR + PS.rotations) - PS.MechOffset;
    PS.MechPositionFixed = PS.MechPosition + PS.MechOffset;

    float elec = ((2.0f * PI / (float)_CPR) * ((int)(NPP * angle) % _CPR)) + PS.ElecOffset;
    if (elec < 0)
    {
        elec += 2.0f * PI;
    }   
    else if (elec > 2.0f * PI)
    {
        elec -= 2.0f * PI;
    }
    PS.ElecPosition = elec;

    float vel;
    if ((PS.position - PS.oldPosition) < -3.0f)
    {
        vel = (PS.position - PS.oldPosition - 2.0f * PI) / dt;
    }
    else if ((PS.position - PS.oldPosition) < -3.0f)
    {
        vel = (PS.position - PS.oldPosition + 2.0f * PI) / dt;
    }
    else
    {
        vel = (PS.position - PS.oldPosition) / dt;
    }

    //average filter
    int n = 40;
    float sum = vel;
    for (int i = 1; i < (n); i++)
    {
        PS.velVec[n - i] = PS.velVec[n - i - 1];
        sum += PS.velVec[n - i];
    }
    PS.velVec[0] = vel;

    PS.MechVelocity = sum / ((float)n);
    PS.ElecVelocity = PS.MechVelocity * NPP;
    PS.ElecVeloFilt = 0.99f * PS.ElecVeloFilt + 0.01f * PS.ElecVelocity;
}

void LUT_Write(int new_lut[128])
{
    memcpy(PS.offset_lut, new_lut, sizeof(PS.offset_lut));
}

