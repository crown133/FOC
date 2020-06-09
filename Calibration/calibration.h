#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "position.h"
#include "structs.h"

#define V_CAL 0.08f;


extern void order_phases(PositionSensor *ps, ControllerStruct *controller);
extern void calibrate(PositionSensor *ps, ControllerStruct *controller);

#endif

