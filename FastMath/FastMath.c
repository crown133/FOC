#include "FastMath.h"
#include "math.h"


const float Multiplier = 81.4873308631f;
const float SinTable[] = 
{
    0,0.012296,0.024589,0.036879,0.049164,0.061441,0.073708,0.085965,0.098208,0.11044,
    0.12265,0.13484,0.14702,0.15917,0.17129,0.18339,0.19547,0.20751,0.21952,0.2315,0.24345,
    0.25535,0.26722,0.27905,0.29084,0.30258,0.31427,0.32592,0.33752,0.34907,0.36057,0.37201,
    0.38339,0.39472,0.40599,0.41719,0.42834,0.43941,0.45043,0.46137,0.47224,0.48305,0.49378,
    0.50443,0.51501,0.52551,0.53593,0.54627,0.55653,0.5667,0.57679,0.58679,0.5967,0.60652,0.61625,
    0.62589,0.63543,0.64488,0.65423,0.66348,0.67263,0.68167,0.69062,0.69946,0.70819,0.71682,0.72534,
    0.73375,0.74205,0.75023,0.75831,0.76626,0.77411,0.78183,0.78944,0.79693,0.80429,0.81154,0.81866,
    0.82566,0.83254,0.83928,0.84591,0.8524,0.85876,0.865,0.8711,0.87708,0.88292,0.88862,0.89419,0.89963,
    0.90493,0.9101,0.91512,0.92001,0.92476,0.92937,0.93384,0.93816,0.94235,0.94639,0.95029,0.95405,0.95766,
    0.96113,0.96445,0.96763,0.97066,0.97354,0.97628,0.97887,0.98131,0.9836,0.98574,0.98774,0.98958,0.99128,0.99282,
    0.99422,0.99546,0.99656,0.9975,0.99829,0.99894,0.99943,0.99977,0.99996,1,0.99988,0.99962,0.9992,0.99863,0.99792,
    0.99705,0.99603,0.99486,0.99354,0.99207,0.99045,0.98868,0.98676,0.98469,0.98247,0.9801,0.97759,0.97493,0.97212,
    0.96916,0.96606,0.96281,0.95941,0.95587,0.95219,0.94836,0.94439,0.94028,0.93602,0.93162,0.92708,0.9224,0.91758,
    0.91263,0.90753,0.9023,0.89693,0.89142,0.88579,0.88001,0.87411,0.86807,0.8619,0.8556,0.84917,0.84261,0.83593,
    0.82911,0.82218,0.81512,0.80793,0.80062,0.7932,0.78565,0.77798,0.7702,0.7623,0.75428,0.74615,0.73791,0.72956,
    0.72109,0.71252,0.70384,0.69505,0.68616,0.67716,0.66806,0.65886,0.64956,0.64017,0.63067,0.62108,0.6114,0.60162,
    0.59176,0.5818,0.57176,0.56163,0.55141,0.54111,0.53073,0.52027,0.50973,0.49911,0.48842,0.47765,0.46682,0.45591,
    0.44493,0.43388,0.42277,0.4116,0.40036,0.38906,0.37771,0.36629,0.35483,0.3433,0.33173,0.32011,0.30843,0.29671,
    0.28495,0.27314,0.26129,0.2494,0.23748,0.22552,0.21352,0.20149,0.18943,0.17735,0.16523,0.15309,0.14093,0.12875,
    0.11655,0.10432,0.092088,0.079838,0.067576,0.055303,0.043022,0.030735,0.018443,0.0061479,-0.0061479,-0.018443,
    -0.030735,-0.043022,-0.055303,-0.067576,-0.079838,-0.092088,-0.10432,-0.11655,-0.12875,-0.14093,-0.15309,-0.16523,
    -0.17735,-0.18943,-0.20149,-0.21352,-0.22552,-0.23748,-0.2494,-0.26129,-0.27314,-0.28495,-0.29671,-0.30843,-0.32011,
    -0.33173,-0.3433,-0.35483,-0.36629,-0.37771,-0.38906,-0.40036,-0.4116,-0.42277,-0.43388,-0.44493,-0.45591,-0.46682,
    -0.47765,-0.48842,-0.49911,-0.50973,-0.52027,-0.53073,-0.54111,-0.55141,-0.56163,-0.57176,-0.5818,-0.59176,-0.60162,
    -0.6114,-0.62108,-0.63067,-0.64017,-0.64956,-0.65886,-0.66806,-0.67716,-0.68616,-0.69505,-0.70384,-0.71252,-0.72109,
    -0.72956,-0.73791,-0.74615,-0.75428,-0.7623,-0.7702,-0.77798,-0.78565,-0.7932,-0.80062,-0.80793,-0.81512,-0.82218,
    -0.82911,-0.83593,-0.84261,-0.84917,-0.8556,-0.8619,-0.86807,-0.87411,-0.88001,-0.88579,-0.89142,-0.89693,-0.9023,
    -0.90753,-0.91263,-0.91758,-0.9224,-0.92708,-0.93162,-0.93602,-0.94028,-0.94439,-0.94836,-0.95219,-0.95587,-0.95941,
    -0.96281,-0.96606,-0.96916,-0.97212,-0.97493,-0.97759,-0.9801,-0.98247,-0.98469,-0.98676,-0.98868,-0.99045,-0.99207,
    -0.99354,-0.99486,-0.99603,-0.99705,-0.99792,-0.99863,-0.9992,-0.99962,-0.99988,-1,-0.99996,-0.99977,-0.99943,-0.99894,
    -0.99829,-0.9975,-0.99656,-0.99546,-0.99422,-0.99282,-0.99128,-0.98958,-0.98774,-0.98574,-0.9836,-0.98131,-0.97887,
    -0.97628,-0.97354,-0.97066,-0.96763,-0.96445,-0.96113,-0.95766,-0.95405,-0.95029,-0.94639,-0.94235,-0.93816,-0.93384,
    -0.92937,-0.92476,-0.92001,-0.91512,-0.9101,-0.90493,-0.89963,-0.89419,-0.88862,-0.88292,-0.87708,-0.8711,-0.865,-0.85876,
    -0.8524,-0.84591,-0.83928,-0.83254,-0.82566,-0.81866,-0.81154,-0.80429,-0.79693,-0.78944,-0.78183,-0.77411,-0.76626,
    -0.75831,-0.75023,-0.74205,-0.73375,-0.72534,-0.71682,-0.70819,-0.69946,-0.69062,-0.68167,-0.67263,-0.66348,-0.65423,
    -0.64488,-0.63543,-0.62589,-0.61625,-0.60652,-0.5967,-0.58679,-0.57679,-0.5667,-0.55653,-0.54627,-0.53593,-0.52551,
    -0.51501,-0.50443,-0.49378,-0.48305,-0.47224,-0.46137,-0.45043,-0.43941,-0.42834,-0.41719,-0.40599,-0.39472,-0.38339,
    -0.37201,-0.36057,-0.34907,-0.33752,-0.32592,-0.31427,-0.30258,-0.29084,-0.27905,-0.26722,-0.25535,-0.24345,-0.2315,
    -0.21952,-0.20751,-0.19547,-0.18339,-0.17129,-0.15917,-0.14702,-0.13484,-0.12265,-0.11044,-0.098208,-0.085965,-0.073708,
    -0.061441,-0.049164,-0.036879,-0.024589,-0.012296,0
};

float FastSin(float theta)
{
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;    
    return SinTable[(int) (Multiplier*theta)] ;
}
    
float FastCos(float theta)
{
    return FastSin(1.57079632679f - theta);
}


float roundf(float x)
{
    /// Returns nearest integer ///
    
    return x < 0.0f ? ceilf(x - 0.5f) : floorf(x + 0.5f);
}
    
void limit_norm(float *x, float *y, float limit)
{
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit)
    {
        *x = *x * limit/norm;
        *y = *y * limit/norm;
    }
}

//计算浮点值在浮点间隔内的确切位置（作为0-1值;您也可以将其视为百分比）。 然后相应地缩放最大整数值。
int float_to_uint(float x, float x_min, float x_max, uint8_t bits)  
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
    
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

