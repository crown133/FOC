/* Includes ------------------------------------------------------------------*/
#include "led.h"


void led_Init(void)
{

      GPIO_InitTypeDef GPIO_InitStruct = {0};

      /* GPIO Ports Clock Enable */
      __HAL_RCC_GPIOB_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOH_CLK_ENABLE();

      /*Configure GPIO pin Output Level */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

      /*Configure GPIO pin : PC13 */
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      
      /*Configure GPIO pin : PB12 */ //testing the loop frequency
//      GPIO_InitStruct.Pin = GPIO_PIN_12;
//      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//      GPIO_InitStruct.Pull = GPIO_PULLUP;
//      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
