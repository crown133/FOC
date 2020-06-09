#include "TLE5012B.h"
#include "spi.h"
#include "delay.h"


void SPI5012_Transmit(u16 byte)
{
    u16 pData[1] = {0};
    pData[0] = byte;
    HAL_SPI_Transmit(&hSPI5012, (u8 *)pData, 1, 10);
}

u16 SPI5012_Receive(void)
{
    u16 pData[1] = {0};
    HAL_SPI_Receive(&hSPI5012, (u8 *)pData, 1, 10);
    return pData[0];
}

u16 TLE5012_ReadValue(u16 command)
{
    u16 RXdata;
    SPI5012_CS_ENABLE();

    SPI5012_Transmit(command);
    RXdata = SPI5012_Receive();
    SPI5012_CS_DISABLE();

    return RXdata;
}

void TLE5012B_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0}; //CS PIN Init
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    SPI5012_CS_DISABLE();

    SPI5012_INIT(); //spi init
}

void TLE5012B_change_angleDir(void) //see page24 of user manual for detail
{
    SPI5012_Transmit(WRITE_MOD2_VALUE);
    SPI5012_Transmit(0x0808); //失能Autocali
    delay_us(128);

    SPI5012_Transmit(WRITE_MOD2_VALUE);
    SPI5012_Transmit(0x0809); //顺时针为正，使能Autocali
    delay_us(128);

    TLE5012_ReadValue(READ_STATUS); //clear S_FUSE
}


