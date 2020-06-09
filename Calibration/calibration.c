#include "calibration.h"
#include "foc.h"
#include "user_config.h"
#include "motor_config.h"
#include "current_controller_config.h"
#include "position.h"
#include "tim.h"
#include "TLE5012B.h"

#include "delay.h"
#include "math.h"
#include "stdlib.h"

///Checks phase order, to ensure that positive Q current produces
///torque in the positive direction wrt the position sensor.
void order_phases(PositionSensor *ps, ControllerStruct *controller)
{
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL; //Put all volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    int sample_counter = 0;

    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation
    for (int i = 0; i < 15000; i++)
    {
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u); // Set duty cycles
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);

        delay_us(100);
    }

    float theta_start;
    /// Rotate voltage angle
    while (theta_ref < 4 * PI) //rotate for 2 electrical cycles
    {
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation
        delay_us(100);
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u); //Set duty cycles
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);
        PS_Value(DT); //sample position sensor
        theta_actual = PS.MechPosition;
        if (theta_ref == 0)
        {
            theta_start = theta_actual;
        }
        theta_ref += 0.001f;
    }
    float theta_end = PS.MechPosition;
    int direction = (theta_end - theta_start) > 0;

    PHASE_ORDER = direction;
}


//float mean = 0;
float error_f[2560] = {0};
float error_b[2560] = {0};
float error[2560] = {0};
float error_filt[2560] = {0};
/// Measures the electrical angle offset of the position sensor
/// and (in the future) corrects nonlinearity due to position sensor eccentricity
void calibrate(PositionSensor *ps, ControllerStruct *controller)
{
    //	  float *error_f = NULL;
    //    float *error_b = NULL;
    //    int *lut = NULL;
    //    int *raw_f = NULL;
    //    int *raw_b = NULL;
    //    float *error = NULL;
    //    float *error_filt = NULL;

    const int n = 128 * NPP;               // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                     // increments between saved samples (for smoothing motion)
    float delta = 2 * PI * NPP / (n * n2); // change in angle between samples

//    static float error_f[n]; // error vector rotating forwards
//    static float error_b[n]; // error vector rotating backwards

    static int lut[n]; // clear any old lookup table before starting.
    static int raw_f[n];
    static int raw_b[n];
//    static float error[n];
//    static float error_filt[n];
    //    error_f = (float*)malloc(n*sizeof(float));//new float[n](); // error vector rotating forwards
    //    error_b = (float*)malloc(n*sizeof(float)); // error vector rotating backwards

    const int n_lut = 128;
    //    lut = (int*)malloc(n_lut*sizeof(int)); // clear any old lookup table before starting.
    //    raw_f = (int*)malloc(n*sizeof(int));
    //	  raw_b = (int*)malloc(n*sizeof(int));
    //    error = (float*)malloc(n*sizeof(int));
    //    error_filt = (float*)malloc(n*sizeof(int));

    const int window = 128;
    float cogging_current[window];

    LUT_Write(lut);

    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL; // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;

    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); // space vector modulation
    for (int i = 0; i < 20000; i++)
    {
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u); // Set duty cycles
        if (PHASE_ORDER)
        {                                                 // Check which phase order to use,
            TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v); // Write duty cycles
            TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);
        }
        else
        {
            TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_v);
            TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_w);
        }
        delay_us(100);
    }
    PS_Value(DT);
    controller->i_b = I_SCALE * (float)(controller->adc2_raw - controller->adc2_offset); //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE * (float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q); //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));

    for (int i = 0; i < n; i++)
    { // rotate forwards
        for (int j = 0; j < n2; j++)
        {
            theta_ref += delta;
            abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      // inverse dq0 transform on voltages
            svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); // space vector modulation
            TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u);
            if (PHASE_ORDER)
            {                                                 // Check phase ordering
                TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v); // Set duty cycles
                TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);
            }
            else
            {
                TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_v);
                TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_w);
            }
            delay_us(100);
            PS_Value(DT);
        }
        PS_Value(DT);
        theta_actual = PS.MechPositionFixed;
        error_f[i] = theta_ref / NPP - theta_actual;
        raw_f[i] = PS.raw_angle;
    }

    for (int i = 0; i < n; i++)
    { // rotate backwards
        for (int j = 0; j < n2; j++)
        {
            theta_ref -= delta;
            abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      // inverse dq0 transform on voltages
            svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); // space vector modulation
            TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u);
            if (PHASE_ORDER)
            {
                TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v);
                TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);
            }
            else
            {
                TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_v);
                TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_w);
            }
            delay_us(100);
            PS_Value(DT);
        }
        PS_Value(DT);                        // sample position sensor
        theta_actual = PS.MechPositionFixed; // get mechanical position
        error_b[i] = theta_ref / NPP - theta_actual;
        raw_b[i] = PS.raw_angle;
    }

    float offset = 0;
    for (int i = 0; i < n; i++)
    {
        offset += (error_f[i] + error_b[n - 1 - i]) / (2.0f * n); // calclate average position sensor offset
    }
    offset = fmod(offset * NPP, 2 * PI); // convert mechanical angle to electrical angle

    PS.ElecOffset = offset; // Set position sensor offset
    E_OFFSET = offset;

    /// Perform filtering to linearize position sensor eccentricity
    /// FIR n-sample average, where n = number of samples in one electrical cycle
    /// This filter has zero gain at electrical frequency and all integer multiples
    /// So cogging effects should be completely filtered out.
    float mean = 0;
    for (int i = 0; i < n; i++)
    { //Average the forward and back directions
        error[i] = 0.5f * (error_f[i] + error_b[n - i - 1]);
    }
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < window; j++)
        {
            int ind = -window / 2 + j + i; // Indexes from -window/2 to + window/2
            if (ind < 0)
            {
                ind += n;
            } // Moving average wraps around
            else if (ind > n - 1)
            {
                ind -= n;
            }
            error_filt[i] += error[ind] / (float)window;
        }
        if (i < window)
        {
            cogging_current[i] = current * sinf((error[i] - error_filt[i]) * NPP);
        }

        mean += error_filt[i] / n;
    }

    int raw_offset = (raw_f[0] + raw_b[n - 1]) / 2; //Insensitive to errors in this direction, so 2 points is plenty

    for (int i = 0; i < n_lut; i++)
    { // build lookup table
        int ind = (raw_offset >> 8) + i;
        if (ind > (n_lut - 1))
        {
            ind -= n_lut;
        }
        lut[ind] = (int)((error_filt[i * NPP] - mean) * (float)(_CPR) / (2.0f * PI));

        delay_us(10);
    }

    LUT_Write(lut); // write lookup table to position sensor object
    //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
    memcpy(&ENCODER_LUT, lut, sizeof(lut)); // copy the lookup table to the flash array


    //    flash_write();// write offset and lookup table to flash

    //		free(error_f);
    //		free(error_b);
    //		free(lut);
    //		free(raw_f);
    //		free(raw_b);
    //		free(error);
    //		free(error_filt);
}

void mechzero_set(void)
{
    PS_Value(DT);
    M_OFFSET = PS.MechPositionFixed;
    PS.MechOffset = PS.MechPositionFixed;
}
