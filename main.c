/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#include "spi.h"
#include "tim.h"
#include "led.h"
#include "delay.h"
#include "adc.h"

#include "TLE5012B.h"
#include "position.h"
#include "DRV8323.h"

#include "FastMath.h"
#include "foc.h"
#include "flash.h"
#include "motor_config.h"
#include "hw_config.h"
#include "user_config.h"
#include "calibration.h"
#include "current_controller_config.h"

////////// state machine ///////////
#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5


u8 state = 0;
volatile u8 state_change;
u8 statef = 0;

int canID_state = 0;
int canid;

int flag_inter = 0;

//////// parameters stored in flash /////////
float __float_reg[64]; // Floats stored in flash
int __int_reg[256];    // Ints stored in flash.  Includes position sensor calibration lookup table


int main(void)
{
    HAL_Init(); //Reset of all peripherals, Initializes the Flash interface and the Systick.

    SystemClock_Config();
    delay_init(168);

    /* Initialize all configured peripherals */
    led_Init();
    led_on;
    delay_us(100);

    SysNVIC_SetPriority();

    flash_load();
    delay_us(100);

    TLE5012B_init(); //Positon Sensor Init
    PS.ElecOffset = E_OFFSET;
    PS.MechOffset = M_OFFSET;
	
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    LUT_Write(lut); // write lookup table to position sensor object
    delay_us(100);

    //  ADC_Init(); //ADC
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    delay_us(100);

    reset_foc(&controller);    // Reset current controller

    delay_us(100);

    drv8323_init(); //MOS Driver Init
    delay_us(100);
    zero_current(&controller.adc1_offset, &controller.adc2_offset); // Measure current sensor zero-offset
    delay_us(100);
    init_controller_params(&controller);
    //    MX_TIM3_Init(); //40kHz timer global interrupt
    //    HAL_TIM_Base_Start_IT(&htim3);
    //    delay_ms(500);

    MX_TIM1_Init(); //PWM output & timer global interrupt
    HAL_TIM_Base_Start_IT(&htim1);
    delay_us(100);

    while (1)
    {
        // delay_ms(10);
			//change can id
			

    }
}

//Control_Loop which runs in 40KHz
//void TIM3_IRQHandler(void)
void TIM1_UP_TIM10_IRQHandler(void)
{
//		PBout(12) = flag_inter;
//		flag_inter = ~flag_inter;
	
//	if(canID_state)
//			{
//				CAN_ID = canid;
////				memcpy(&ENCODER_LUT, lut, sizeof(lut));
//				flash_write(); //write to flash
//				canID_state = 0;
//				
//			}
			
    ADC1->CR2 |= 0x40000000; // Begin sample and conversion 三重同步模式下只开启adc1转换就能自动开启adc2 3

    PS_Value(DT);
    //	ADC_Filter(&controller);
    controller.theta_mech = (1.0f / GR) * PS.MechPosition;
    controller.theta_elec = PS.ElecPosition;
    controller.dtheta_mech = (1.0f / GR) * PS.MechVelocity;
    controller.dtheta_elec = PS.ElecVelocity;

    // Read ADC Data Registers
    controller.adc1_raw = ADC2->DR; //PC2 SOB
    controller.adc2_raw = ADC3->DR; //PC3 SOA
    controller.adc3_raw = ADC1->DR; //Vbus

    controller.v_bus = 0.95f * controller.v_bus + 0.05f * V_SCALE * (float)controller.adc3_raw;

    // if (statef)
    // {
    //     drv_enable_gd();

    //     float theta_ref = 0;
    //     float theta_actual = 0;
    //     float v_d = 0.1; //Put all volts on the D-Axis
    //     float v_q = 0.0f;
    //     float v_u, v_v, v_w = 0;
    //     float dtc_u, dtc_v, dtc_w = .5f;

    //     ///Set voltage angle to zero, wait for rotor position to settle
    //     abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);      //inverse dq0 transform on voltages
    //     svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w); //space vector modulation

    //     for (int i = 0; i < 20000; i++)
    //     {
    //         TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f - dtc_u); // Set duty cycles
    //         if (PHASE_ORDER)
    //         {                                                 // Check which phase order to use,
    //             TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_v); // Write duty cycles
    //             TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_w);
    //         }
    //         else
    //         {
    //             TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f - dtc_v);
    //             TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f - dtc_w);
    //         }
    //         delay_us(100);
    //     }
    //     drv_disable_gd();
    //     statef = 0;
    // }
    
    /// Check state machine state, and run the appropriate function ///
    switch (state)
    {
    case REST_MODE: // 0 Do nothing
        if (state_change)
        {
            //    enter_menu_state();
        }
        break;
    case CALIBRATION_MODE: // 1 Run encoder calibration procedure
        if (state_change)
        {
            led_off;
            delay_ms(300);
            drv_enable_gd();
            led_on;                         // Turn on status LED
            order_phases(&PS, &controller); // Check phase ordering
            calibrate(&PS, &controller);    // Perform calibration procedure
            drv_disable_gd();
            led_off;       // Turn off status LED
            flash_write(); //write to flash
            delay_ms(300);
            led_on;
            state_change = 0;
        }
        break;
    case MOTOR_MODE: //2 Run torque control
        if (state_change)
        {
            drv_enable_gd();
            led_on;
            reset_foc(&controller);
            reset_observer(&observer, &iq_eso, 5000, 0.025, 3);
            delay_ms(1);
            controller.i_d_ref = 0;
            controller.i_q_ref = 0;

            state_change = 0;
        }
        else
        {

            //  if(controller.v_bus>28.0f){         //Turn of gate drive if bus voltage is too high, to prevent FETsplosion if the bus is cut during regen

            //            if ((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0))
            //            {
            //                controller.i_d_ref = 0;
            //                controller.i_q_ref = 0;
            //                controller.kp = 0;
            //                controller.kd = 0;
            //                controller.t_ff = 0;
            //            }
						iq_eso.beta_01 = 2 * iq_eso.omega;
						iq_eso.beta_02 = iq_eso.omega * iq_eso.omega;
					
//            controller.kp_d = controller.kp_q;
//            controller.ki_d = controller.ki_q;
            torque_control(&controller);
            commutate(&controller, &observer, controller.theta_elec);

            //            controller.timeout++;
        }
        break;
    case SETUP_MODE:
        if (state_change)
        {
            //    enter_setup_state();
        }
        break;
    case ENCODER_MODE:
        //    print_encoder();
        break;

    default:
        break;
    }

    //    HAL_TIM_IRQHandler(&htim3);
    HAL_TIM_IRQHandler(&htim1);
}
