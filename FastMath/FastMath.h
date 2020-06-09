#ifndef __FASTMATH_H
#define __FASTMATH_H

#include "sys.h"

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

//#define fmaxf(x, y)         (((x)>(y))?(x):(y))
//#define fminf(x, y)         (((x)<(y))?(x):(y))
#define fmaxf3(x, y, z)     ((x) > (y) ? ((x) > (z) ? (x) : (z)) : ((y) > (z) ? (y) : (z)))
#define fminf3(x, y, z)     ((x) < (y) ? ((x) < (z) ? (x) : (z)) : ((y) < (z) ? (y) : (z)))

float FastSin(float theta);
float FastCos(float theta);
float roundf(float x);
void limit_norm(float *x, float *y, float limit);
int float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits);




#endif
