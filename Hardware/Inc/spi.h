#ifndef __spi_H
#define __spi_H

#include "sys.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;


void MX_SPI1_Init(void);     //DRV8323
void MX_SPI3_Init(void);


#endif /*__ spi_H */
