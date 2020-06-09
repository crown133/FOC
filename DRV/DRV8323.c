#include "DRV8323.h"
#include "spi.h"
#include "delay.h"
#include "sys.h"
#include "foc.h"


static void drv8323_pin_init(void)
{
    GPIO_CS_CLK_EN();
    GPIO_DRVEN_CLK_EN();

    //CS PIN Init
    GPIO_InitTypeDef GPIO_InitStruct = {0};  
    GPIO_InitStruct.Pin = GPIO_PIN_CS;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_CS, &GPIO_InitStruct);  
    SPIDRV_CS_DISABLE();
    
    //Driver Enable PIN Init
    GPIO_InitStruct.Pin = GPIO_PIN_DRVEN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIO_DRVEN, &GPIO_InitStruct);

}

static u16 drv8323_write(u16 byte)
{
    u16 TXdata[1] = {0};
    u16 RXdata[1] = {0};
    TXdata[0] = byte;

    SPIDRV_CS_ENABLE();
    delay_us(10);
    HAL_SPI_TransmitReceive(&hspi1, (u8 *)TXdata, (u8 *)RXdata, 1, 10);
    SPIDRV_CS_DISABLE();

    return RXdata[0];
}

///////////////////////////////////
static void drv_write_register(u16 reg, u16 val)    //bit15: 0 write
{
    drv8323_write((reg << 11) | val);
}

u16 drv_read_register(u16 reg)           //bit15: 1 read
{
    return drv8323_write((1<<15) | (reg<<11));
}

////////////////////////////////////
u16 drv_read_FSReg1(void)
{
    return drv8323_write((1 << 15) | (FSReg1<<11));
}

u16 drv_read_FSReg2(void)
{
    return drv8323_write((1 << 15) | (FSReg2<<11));
}


void drv_write_DCReg(u16 DIS_CPUV, u16 DIS_GDF, u16 OTW_REP, u16 PWM_MODE, u16 PWM_COM, u16 PWM_DIR, u16 COAST, u16 BRAKE, u16 CLR_FLT)
{
    u16 val = (DCReg << 11) | (DIS_CPUV << 9) | (DIS_GDF << 8) | (OTW_REP << 7) | (PWM_MODE << 5) | (PWM_COM << 4) | (PWM_DIR << 3) | (COAST << 2) | (BRAKE << 1) | CLR_FLT;
    drv8323_write(val);
}

void drv_write_HSReg(u16 LOCK, u16 IDRIVEP_HS, u16 IDRIVEN_HS)
{
    u16 val = (HSReg << 11) | (LOCK << 8) | (IDRIVEP_HS << 4) | IDRIVEN_HS;
    drv8323_write(val);
}

void drv_write_LSReg(u16 CBC, u16 TDRIVE, u16 IDRIVEP_LS, u16 IDRIVEN_LS)
{
    u16 val = (LSReg << 11) | (CBC << 10) | (TDRIVE << 8) | (IDRIVEP_LS << 4) | IDRIVEN_LS;
    drv8323_write(val);
}

void drv_write_OCPCReg(u16 TRETRY, u16 DEAD_TIME, u16 OCP_MODE, u16 OCP_DEG, u16 VDS_LVL)
{
    u16 val = (OCPCReg << 11) | (TRETRY << 10) | (DEAD_TIME << 8) | (OCP_MODE << 6) | (OCP_DEG << 4) | VDS_LVL;
    drv8323_write(val);
}

void drv_write_CSACReg(u16 CSA_FET, u16 VREF_DIV, u16 LS_REF, u16 CSA_GAIN, u16 DIS_SEN, u16 CSA_CAL_A, u16 CSA_CAL_B, u16 CSA_CAL_C, u16 SEN_LVL)
{
    u16 val = (CSACReg << 11) | (CSA_FET << 10) | (VREF_DIV << 9) | (LS_REF << 8) | (CSA_GAIN << 6) | (DIS_SEN << 5) | (CSA_CAL_A << 4) | (CSA_CAL_B << 3) | (CSA_CAL_C << 2) | SEN_LVL;
    drv8323_write(val);
}

void drv_print_faults(void)
{
    u16 val1 = drv_read_FSReg1();
    delay_us(10);
    u16 val2 = drv_read_FSReg2();
    delay_us(10);

 //   if(val1 & (1<<10)){printf("\n\rFAULT\n\r");}
//
 //   if(val1 & (1<<9)){printf("VDS_OCP\n\r");}
 //   if(val1 & (1<<8)){printf("GDF\n\r");}
 //   if(val1 & (1<<7)){printf("UVLO\n\r");}
 //   if(val1 & (1<<6)){printf("OTSD\n\r");}
 //   if(val1 & (1<<5)){printf("VDS_HA\n\r");}
 //   if(val1 & (1<<4)){printf("VDS_LA\n\r");}
 //   if(val1 & (1<<3)){printf("VDS_HB\n\r");}
 //   if(val1 & (1<<2)){printf("VDS_LB\n\r");}
 //   if(val1 & (1<<1)){printf("VDS_HC\n\r");}
 //   if(val1 & (1)){printf("VDS_LC\n\r");}
 //   
 //   if(val2 & (1<<10)){printf("SA_OC\n\r");}
 //   if(val2 & (1<<9)){printf("SB_OC\n\r");}
 //   if(val2 & (1<<8)){printf("SC_OC\n\r");}
 //   if(val2 & (1<<7)){printf("OTW\n\r");}
 //   if(val2 & (1<<6)){printf("CPUV\n\r");}
 //   if(val2 & (1<<5)){printf("VGS_HA\n\r");}
 //   if(val2 & (1<<4)){printf("VGS_LA\n\r");}
 //   if(val2 & (1<<3)){printf("VGS_HB\n\r");}
 //   if(val2 & (1<<2)){printf("VGS_LB\n\r");}
 //   if(val2 & (1<<1)){printf("VGS_HC\n\r");}
 //   if(val2 & (1)){printf("VGS_LC\n\r");}
}

void drv_enable_gd(void)
{
    u16 val = (drv_read_register(DCReg)) & (~(0x1 << 2));
    drv_write_register(DCReg, val);
}

void drv_disable_gd(void)
{
    u16 val = (drv_read_register(DCReg)) | (0x1 << 2);
    drv_write_register(DCReg, val);
}

void drv_calibrate(void)
{
    u16 val = (0x1 << 4) + (0x1 << 3) + (0x1 << 2);
    drv_write_register(CSACReg, val);
}

void drv8323_init(void)
{
    drv8323_pin_init();
    delay_us(100);
    DRIVER_ENABLE();  //
    SPIDRV_INIT();
    delay_us(100);
    drv_write_HSReg(LOCK_OFF, IDRIVEP_HS_1000MA, IDRIVEN_HS_2000MA);
    delay_us(100);
    drv_calibrate();
    delay_us(100);
    drv_write_DCReg(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    delay_us(100);    
    drv_write_CSACReg(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0); //I sense gain = 40  (V = 40*0.001*I)
    delay_us(100);
    drv_write_OCPCReg(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);
    delay_us(100);
    // drv_enable_gd();
//    zero_current(&controller.adc1_offset, &controller.adc2_offset);    // Measure current sensor zero-offset  
    delay_us(10);
    drv_disable_gd();
}
