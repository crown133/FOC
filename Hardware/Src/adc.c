#include "adc.h"


ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

// uint32_t adc_buf[CH_NUM];

// uint32_t adc_val[3];

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)*/
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_BUS_CHANNEL;     
  sConfig.Rank = 1;                               // adc采样时间 = 转换时间 + 读取时间 
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // 转换时间 = 采样时间 + 12.5 adc周期
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  __HAL_ADC_ENABLE(&hadc1);
  
}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_PHASE1_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_ADC_ENABLE(&hadc2);

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_PHASE2_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_ADC_ENABLE(&hadc3);

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration   Bus Voltage 
    PC0     ------> ADC1_IN10 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  }
  else if(adcHandle->Instance==ADC2)
  {
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration    
    PC2     ------> ADC2_IN12 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  }
  else if(adcHandle->Instance==ADC3)
  {
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration    
    PC3     ------> ADC3_IN13  */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance==ADC1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC1     ------> ADC1_IN11   */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);

  }
  else if(adcHandle->Instance==ADC2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PC2     ------> ADC2_IN12  */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);

  }
  else if(adcHandle->Instance==ADC3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PC3     ------> ADC3_IN13 */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

  }
} 

void ADC_Init(void)
{
    void MX_ADC1_Init(void);
    void MX_ADC2_Init(void);
    void MX_ADC3_Init(void);
}

// /* ADC1 init function */
// void MX_ADC1_Init(void)
// {
//     ADC_ChannelConfTypeDef sConfig;
    
//     HAL_ADC_Stop_DMA(&hadc1);
//   /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)  */
//     hadc1.Instance = ADC1;
//     hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//     hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//     hadc1.Init.ScanConvMode = ENABLE;
//     hadc1.Init.ContinuousConvMode = ENABLE;
//     hadc1.Init.DiscontinuousConvMode = DISABLE;
//     hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//     hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//     hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//     hadc1.Init.NbrOfConversion = CH_NUM;
//     hadc1.Init.DMAContinuousRequests = ENABLE;
//     hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
//     if (HAL_ADC_Init(&hadc1) != HAL_OK)
//     {
//         Error_Handler();
//     }
//     /* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.  */
//     sConfig.Channel = ADC_CHANNEL_10;                   //adc采样时间 = 转换时间 + 读取时间  
//     sConfig.Rank = 1;                                   //转换时间 = 采样时间 + 12.5 adc周期
//     sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//     if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     sConfig.Channel = ADC_CHANNEL_12;
//     sConfig.Rank = 2;
//     sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//     if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     sConfig.Channel = ADC_CHANNEL_13;
//     sConfig.Rank = 3;
//     sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

//     if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     HAL_ADC_Start_DMA(&hadc1, adc_buf, CH_NUM * ADC_SAMPLE_NUM); //开启第一次DMA传送
// }

// void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
// {

//     GPIO_InitTypeDef GPIO_InitStruct;
//     if (adcHandle->Instance == ADC1)
//     {

//         /* ADC1 clock enable */
//         __HAL_RCC_ADC1_CLK_ENABLE();

//         /**ADC1 GPIO Configuration
//         PC0     ------> ADC1_IN10 //母线电压
//         PC2     ------> ADC1_IN12
//         PC3     ------> ADC1_IN13    
//         */
//         GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
//         GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//         GPIO_InitStruct.Pull = GPIO_NOPULL;
//         HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        
//         /* ADC1 DMA Init */
//         __HAL_RCC_DMA2_CLK_ENABLE();        //f4 无adc校准

//         hdma_adc1.Instance = DMA2_Stream0;
//         hdma_adc1.Init.Channel = DMA_CHANNEL_0;
//         hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//         hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//         hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//         hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//         hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//         hdma_adc1.Init.Mode = DMA_CIRCULAR;
//         hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
//         hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//         if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
//         {
//             Error_Handler();
//         }
        
//         HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);  //cube 默认开启dma中断，会调用回调函数，浪费时间
//         __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
//     }
// }

// void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
// {

//     if (adcHandle->Instance == ADC1)
//     {

//         /* Peripheral clock disable */
//         __HAL_RCC_ADC1_CLK_DISABLE();

//         /**ADC1 GPIO Configuration
//         PC0     ------> ADC1_IN10 //母线电压
//         PC2     ------> ADC1_IN12
//         PC3     ------> ADC1_IN13    
//         */
//         HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

//         /* ADC1 DMA DeInit */
//         HAL_DMA_DeInit(adcHandle->DMA_Handle);

//         /* ADC1 interrupt Deinit */
//         HAL_NVIC_DisableIRQ(ADC_IRQn);
//     }
// }

// //ADC均值滑动滤波
// void ADC_Filter(ControllerStruct* controller)
// {
//     uint16_t i;
//     uint32_t sum[CH_NUM] = {0};

//     for (i = 0; i < ADC_SAMPLE_NUM; i++)
//     {
//         sum[0] += adc_buf[CH_NUM * i + 0];
//         sum[1] += adc_buf[CH_NUM * i + 1];
//         sum[2] += adc_buf[CH_NUM * i + 2];
//     }
//     controller->adc3_raw = sum[0] / ADC_SAMPLE_NUM; //Voltage
//     controller->adc2_raw = sum[1] / ADC_SAMPLE_NUM; //phase B
//     controller->adc1_raw = sum[2] / ADC_SAMPLE_NUM; //phase A
// }


