#ifndef __STRUCTS_H
#define __STRUCTS_H

#include "sys.h"
    
typedef struct
{
    int adc1_raw, adc2_raw, adc3_raw;                       // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus;                                            // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float kp_d, kp_q, ki_d, ki_q, alpha;                      // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc1_offset, adc2_offset;                           // ADC offsets
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    float p_des, v_des, kp, kd, t_ff;                       // Desired position, velocity, gains, torque
    float v_ref, fw_int;                                    // output voltage magnitude, field-weakening integral
    float cogging[128];
} ControllerStruct;

typedef struct
{
	/******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
		float h;  //����
			
		float omega; //ϵͳ����
		float z1;
		float z2;
		float z3;//���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
		float e;//ϵͳ״̬���
		float beta_01;
		float beta_02;
		float beta_03;
		float b0;//�Ŷ�����	
		float u;//���Ŷ�����������
} LESO;

typedef struct
{
    double temperature;     // Estimated temperature
    double temperature2;
    float resistance;
		float z1;
		float z2;
		float u;
} ObserverStruct;

#endif

