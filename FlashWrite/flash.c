#include "flash.h"
#include "user_config.h"

#include "delay.h"

//由工程生成的 .map 文件可以看出整个程序所占大小为16.41KB，
//为保险起见，程序不被flash操作擦除，将数据从第7扇区开始存起
//64+256 Bytes，故需要7扇区
#define __sector 7

///read flash
static int flashReadInt(uint32_t sector, uint32_t index)
{
    return *(int *)(__SECTOR_ADDRS[sector] + 4 * index);
}

static uint32_t flashReadUint(uint32_t sector, uint32_t index)
{
    return *(uint32_t *)(__SECTOR_ADDRS[sector] + 4 * index);
}

static float flashReadFloat(uint32_t sector, uint32_t index)
{
    return *(float *)(__SECTOR_ADDRS[sector] + 4 * index);
}

////init flash and load parameters
void flash_load(void)
{
    __HAL_FLASH_SET_LATENCY(5); //168Mhz votage = 3.3v
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    int offs;
    for (offs = 0; offs < 256; offs++)
    {
        __int_reg[offs] = flashReadInt(__sector, offs);
    }
    for (; offs < 320; offs++)
    {
        __float_reg[offs - 256] = flashReadFloat(__sector, offs);
    }
}

uint32_t sectorError; //the number of error sector
void flash_write(void)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_DATA_CACHE_DISABLE(); //disable data cache when erase sector

    //erase sector 7
    FLASH_EraseInitTypeDef EraseInit;
    EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInit.Banks = FLASH_BANK_1;
    EraseInit.Sector = FLASH_SECTOR_7;
    EraseInit.NbSectors = 1; //sector 7
    EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    int offs;
    for (offs = 0; offs < 320; offs++) //it can not be erased for once time
    {
        if (flashReadUint(__sector, offs) != 0xFFFFFFFF)
        {
            HAL_FLASHEx_Erase(&EraseInit, &sectorError);
        }
    }
    delay_us(500);

    for (offs = 0; offs < 256; offs++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_SECTOR_7 + 4 * offs, __int_reg[offs]);
        delay_us(10);
    }
    for (; offs < 320; offs++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_SECTOR_7 + 4 * offs, *(volatile uint32_t *)(__float_reg + offs - 256));
        delay_us(10);
    }

    __HAL_FLASH_DATA_CACHE_ENABLE();
    HAL_FLASH_Lock();
}
