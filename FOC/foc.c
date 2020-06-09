#include "foc.h"
#include "motor_config.h"
#include "user_config.h"

ControllerStruct controller;
ObserverStruct observer;
LESO iq_eso;

/// Inverse DQ0 Transform ///
void abc(float theta, float d, float q, float *a, float *b, float *c)
{
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q; // Faster Inverse DQ0 transform
    *b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
    *c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}

/// Space Vector Modulation ///
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    /// u,v,w amplitude = v_bus for full modulation depth ///

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w)) / 2.0f;
    *dtc_u = fminf(fmaxf(((u - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
}

////七段式SVPWM//
//void SvpwmCommutation(float vd,float vq,float eleangle)//electrical degree
//{
//	float u_alpha, u_beta;
//	float Uref1, Uref2, Uref3;
//	u8 flagA,flagB,flagC,flagN;
//	float T1,T2,t1,t2;
//	float TS = 1800;

//	float gen3 = 1.732050808f,Kgen3 =259.8076211f;//1800*gen3/12
//
////park逆变化
////	v_alpha = vd*arm_cos_f32(eleangle) - vq*arm_sin_f32(eleangle);
//// 	v_beta  = vd*arm_sin_f32(eleangle) + vq*arm_cos_f32(eleangle);
////	v_alpha = -vq*sinTable_f32[54];
//// 	v_beta  =  vq*sinTable_f32[234];
//	v_alpha = -vq*arm_sin_f32(eleangle);
// 	v_beta  =  vq*arm_cos_f32(eleangle);
//

//	Uref1 = u_beta;
//	Uref2 = ( SQRT3 * u_alpha - u_beta) / 2.0f;
//	Uref3 = (-SQRT3 * u_alpha - u_beta) / 2.0f;
///********************
//以下为判断矢量电压所在扇区
//*********************/
//	if(Uref1>0) {flagA = 1; else flagA = 0;}
//	if(Uref2>0) {flagB = 1; else flagB = 0;}
//	if(Uref3>0) {flagC = 1; else flagC = 0;}
//	flagN = flagA + 2*flagB + 4*flagC;
//

//	switch(flagN)
//	{
//		case 3:	{T1 =  vr2*Kgen3; T2 =  vr1*Kgen3; break;}//sector1 u1,u2>0
//		case 1:	{T1 = -vr2*Kgen3; T2 = -vr3*Kgen3; break;}//sector2 u1>0
//		case 5:	{T1 =  vr1*Kgen3; T2 =  vr3*Kgen3; break;}//sector3 u1,u3>0
//		case 4:	{T1 = -vr1*Kgen3; T2 = -vr2*Kgen3; break;}//sector4 u3>0
//		case 6:	{T1 =  vr3*Kgen3; T2 =  vr2*Kgen3; break;}//sector5 u2,u3>0
//		case 2:	{T1 = -vr3*Kgen3; T2 = -vr1*Kgen3; break;}//sector6 u2>0
//		default: {T1 = 0; T2 = 0; break;}
//	}

//	if (T1+T2>TS)
//	{
//		t1 = T1;
//		t2 = T2;
//		T1 = t1/(t1+t2)*(TS-90);
//		T2 = t2/(t1+t2)*(TS-90);
//	}
//
//	TA = (TS-T1-T2)/2.0f;
//	TB = (TS+T1-T2)/2.0f;
//	TC = (TS+T1+T2)/2.0f;

//	switch(flagN)
//	{
//		case 3:	{APHASE = TA; BPHASE = TB; CPHASE = TC; break;}
//		case 1:	{APHASE = TB; BPHASE = TA; CPHASE = TC; break;}
//		case 5:	{APHASE = TC; BPHASE = TA; CPHASE = TB; break;}
//		case 4:	{APHASE = TC; BPHASE = TB; CPHASE = TA; break;}
//		case 6:	{APHASE = TB; BPHASE = TC; CPHASE = TA; break;}
//		case 2:	{APHASE = TA; BPHASE = TC; CPHASE = TB; break;}
//		default: {APHASE = 1800; BPHASE = 1800; CPHASE = 1800;break;}
//	}
//
//}

/// DQ0 Transform ///
void dq0(float theta, float a, float b, float c, float *d, float *q)
{
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

// Measure zero-offset of the current sensors
void zero_current(int *offset_1, int *offset_2)
{
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i < n; i++)
    {                                         // Average n samples of the ADC
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f); // Write duty cycles
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f);
        ADC1->CR2 |= 0x40000000; // Begin sample and conversion 三重同步模式下只开启adc1转换就能自动开启adc2 3
        delay_us(100);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC3->DR;
    }
    *offset_1 = adc1_offset / n;
    *offset_2 = adc2_offset / n;
}

// linearizes the output of the inverter, which is not linear for small duty cycles ///
void linearize_dtc(float *dtc)
{
    float sgn = 1.0f - (2.0f * (dtc < 0));
    if (fabs(*dtc) >= .01f)
    {
        *dtc = *dtc * .986f + .014f * sgn;
    }
    else
    {
        *dtc = 2.5f * (*dtc);
    }
}

void init_controller_params(ControllerStruct *controller)
{
    //    controller->ki_d = KI_D;
    //    controller->ki_q = KI_Q;
    //    controller->kp_d = K_SCALE * I_BW;
    //    controller->kp_q = K_SCALE * I_BW;
    //    controller->alpha = 1.0f - 1.0f / (1.0f - DT * I_BW * 2.0f * PI);

    controller->ki_d = 0.0015f;
    controller->ki_q = 0.0015f;
    controller->kp_d = 0.026f;
    controller->kp_q = 0.026f;
		controller->kp = 3;
		controller->kd = 1.15;
    //    controller->alpha = 1.0f - 1.0f / (1.0f - DT * I_BW * 2.0f * PI);
}

void reset_foc(ControllerStruct *controller)
{
    TIM1->CCR3 = (PWM_ARR >> 1) * (0.5f);
    TIM1->CCR1 = (PWM_ARR >> 1) * (0.5f);
    TIM1->CCR2 = (PWM_ARR >> 1) * (0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
}

void reset_observer(ObserverStruct *observer, LESO* eso, float b0, float h, float omega)
{
    observer->temperature = 25.0f;
    observer->resistance = .1f;

		eso->h = h;
		eso->b0 = b0;
    eso->omega = omega;
    eso->beta_01 = 2 * eso->omega;
    eso->beta_02 = eso->omega * eso->omega;
}

static inline void LESO_2N(LESO* eso, float y)  
{
    eso->e = y - eso->z1;
    eso->z1 += eso->h * (eso->z2 + eso->beta_01*eso->e + eso->b0*eso->u);
		eso->z2 += eso->h * (eso->beta_02*eso->e);
}

uint8_t vq_adrc = 0;
float kp_q = 0.005;

void commutate(ControllerStruct *controller, ObserverStruct *observer, float theta)
{
    /// Commutation Loop ///
    if (PHASE_ORDER)
    {                                                                                        // Check current sensor ordering
        controller->i_b = I_SCALE * (float)(controller->adc2_raw - controller->adc2_offset); // Calculate phase currents from ADC readings
        controller->i_c = I_SCALE * (float)(controller->adc1_raw - controller->adc1_offset);
    }
    else
    {
        controller->i_c = I_SCALE * (float)(controller->adc2_raw - controller->adc2_offset);
        controller->i_b = I_SCALE * (float)(controller->adc1_raw - controller->adc1_offset);
    }
    controller->i_a = -controller->i_b - controller->i_c;

    float s = FastSin(theta);
    float c = FastCos(theta);
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q); //dq0 transform on currents

    // controller->i_q_filt = 0.95f * controller->i_q_filt + 0.05f * controller->i_q;
    // controller->i_d_filt = 0.95f * controller->i_d_filt + 0.05f * controller->i_d;

    // // Filter the current references to the desired closed-loop bandwidth
    // controller->i_d_ref_filt = (1.0f - controller->alpha) * controller->i_d_ref_filt + controller->alpha * controller->i_d_ref;
    // controller->i_q_ref_filt = (1.0f - controller->alpha) * controller->i_q_ref_filt + controller->alpha * controller->i_q_ref;

    // float scog12 = FastSin(12.0f * theta);
    // float scog1 = s;
    // float cogging_current = 0.25f * scog1 - 0.3f * scog12;

    /// Observer Prediction ///
    LESO_2N(&iq_eso, controller->i_q);
		observer->z1 = iq_eso.z1;
		observer->z2 = iq_eso.z2;
    /// Field Weakening ///
    // controller->fw_int += .001f * (0.5f * OVERMODULATION * controller->v_bus - controller->v_ref);
    // controller->fw_int = fmaxf(fminf(controller->fw_int, 0.0f), -I_FW_MAX);
    // controller->i_d_ref = controller->fw_int;
    //float i_cmd_mag_sq = controller->i_d_ref*controller->i_d_ref + controller->i_q_ref*controller->i_q_ref;
    limit_norm(&controller->i_d_ref, &controller->i_q_ref, I_MAX);

    /// PI Controller ///
    float i_d_error = controller->i_d_ref - controller->i_d;
    float i_q_error = controller->i_q_ref - controller->i_q; //+ cogging_current;

    // float v_d_ff = 2.0f * (controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q_ref); //feed-forward voltages
    // float v_q_ff = 2.0f * (controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d_ref + WB));

    // Integrate Error //
    controller->d_int += controller->kp_d * controller->ki_d * i_d_error;
    controller->q_int += controller->kp_q * controller->ki_q * i_q_error;  //

    controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION * controller->v_bus), -OVERMODULATION * controller->v_bus);
    controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION * controller->v_bus), -OVERMODULATION * controller->v_bus);

    controller->v_d = controller->kp_d * i_d_error + controller->d_int; //+ v_d_ff;
//    controller->v_q = controller->kp_q * i_q_error + controller->q_int; //+ v_q_ff;  //

    iq_eso.u = kp_q * (controller->i_q_ref - iq_eso.z1) - iq_eso.z2 / iq_eso.b0;  //linear state error feedback
		observer->u = iq_eso.u;
//		if(vq_adrc)
		{
				controller->v_q = iq_eso.u;
		}
    controller->v_ref = sqrt(controller->v_d * controller->v_d + controller->v_q * controller->v_q);

    limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION * controller->v_bus);                                                    // Normalize voltage vector to lie within curcle of radius v_bus
    abc(controller->theta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w);                   //inverse dq0 transform on voltages
    svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

    // observer->i_d_dot = 0.5f * (controller->v_d - 2.0f * (observer->i_d_est * R_PHASE - controller->dtheta_elec * L_Q * observer->i_q_est)) / L_D; //feed-forward voltage
    // observer->i_q_dot = 0.5f * (controller->v_q - 2.0f * (observer->i_q_est * R_PHASE + controller->dtheta_elec * (L_D * observer->i_d_est + WB))) / L_Q;

    if (PHASE_ORDER)
    {                                                        // Check which phase order to use,
        TIM1->CCR3 = (PWM_ARR) * (1.0f - controller->dtc_u); // Write duty cycles
        TIM1->CCR2 = (PWM_ARR) * (1.0f - controller->dtc_v);
        TIM1->CCR1 = (PWM_ARR) * (1.0f - controller->dtc_w);
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR) * (1.0f - controller->dtc_u);
        TIM1->CCR1 = (PWM_ARR) * (1.0f - controller->dtc_v);
        TIM1->CCR2 = (PWM_ARR) * (1.0f - controller->dtc_w);
    }

    controller->theta_elec = theta; //For some reason putting this at the front breaks thins

    if (controller->loop_count > 400)
    {
        //controller->i_q_ref = -controller->i_q_ref;
        controller->loop_count = 0;
    }
}

void torque_control(ControllerStruct *controller)
{
    float torque_ref = controller->kp * (controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd * (controller->v_des - controller->dtheta_mech);
    //float torque_ref = -.1*(controller->p_des - controller->theta_mech);
    controller->i_q_ref = torque_ref / KT_OUT;
    controller->i_d_ref = 0.0f;
}
