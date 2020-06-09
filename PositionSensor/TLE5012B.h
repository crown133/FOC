#ifndef __TLE5012B_H
#define __TLE5012B_H

#include "sys.h"

#define SPI5012_TX_OFF              \
    {                               \
        GPIOC->OTYPER &= 0xFFFF;    \
        GPIOC->OTYPER |= (1 << 12); \
    } //PC12(MOSI)=1 setting to OOD mode
#define SPI5012_TX_ON                \
    {                                \
        GPIOC->OTYPER &= 0xFFFF;     \
        GPIOC->OTYPER &= ~(1 << 12); \
    } //PC12(MOSI)=0 setting to OPP mode

/* TLE5012B SPI INIT */
#define SPI5012_INIT() MX_SPI3_Init()
#define hSPI5012 hspi3
#define SPI5012 SPI3

#define SPI5012_CS_ENABLE() PAout(15) = 0
#define SPI5012_CS_DISABLE() PAout(15) = 1
#define INDEX_ENABLE() PAout(15) = 1 /* for incremental signal index */

/* SPI command for TLE5012B */             //RW_Lock_UPD_ADDR_ND  for reference in page 33 of TLE5012B User Manual
#define READ_STATUS 0x8000                 //+CCW/-CW bits8:0 presents for the number of revolutions
#define READ_ANGLE_VALUE 0x8020            //1_0000_0_000010_0001
#define READ_SPEED_VALUE 0x8030            //1_0000_0_000011_0001
#define READ_ANGLE_Revolution_VALUE 0x8041 //1_0000_0_000100_0001
#define READ_TEMPER 0xD051                 //1_1010_0_000101_0001
#define WRITE_MOD1_VALUE 0x5060            //0_1010_0_000110_0001
#define MOD1_VALUE 0x0001
#define WRITE_MOD2_VALUE 0x5081 //0_1010_0_001000_0001
#define MOD2_VALUE 0x0801       //P88 angleRange  é»˜è?¤è?’åº¦èŒƒå›´/ bit3=0 CCWä¸ºæ?? / disable predict / enable autocal mode1
#define WRITE_MOD3_VALUE 0x5091 //0_1010_0_001001_0001
#define MOD3_VALUE 0x0000
#define WRITE_MOD4_VALUE 0x50E1 //0_1010_0_001110_0001
#define MOD4_VALUE 0x0098       //9bit 512

#define WRITE_IFAB_VALUE 0x50B1 //
#define IFAB_VALUE 0x000D

/* Functionality mode */
#define REFERESH_ANGLE 0

////////////////////////
#define _CPR 32768 //counts per rotation



void TLE5012B_init(void);
u16 TLE5012_ReadValue(u16 command);

void TLE5012B_change_angleDir(void);

#endif
