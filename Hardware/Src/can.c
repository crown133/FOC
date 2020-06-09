#include "can.h"
#include "user_config.h"

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;   
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;  //Baud Rate = 42M/((1+BS1+BS2)*Prescaler) = 1M
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;  //ʱ�䴥��ģʽ
  hcan1.Init.AutoBusOff = DISABLE;  //�Զ����߹���ģʽ
  hcan1.Init.AutoWakeUp = DISABLE;  //�Զ�����ģʽ
  hcan1.Init.AutoRetransmission = ENABLE;  //�Զ��ش�ģʽ
  hcan1.Init.ReceiveFifoLocked = DISABLE;  //����fifo����ģʽ
  hcan1.Init.TransmitFifoPriority = ENABLE;  //����fifo���ȼ�
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	
	//  CAN Filter
	CAN_FilterTypeDef Canfilter;
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//filtrate any ID you want here
	Canfilter.FilterIdHigh = 0x0000;
	Canfilter.FilterIdLow = 0x0000;
	Canfilter.FilterMaskIdHigh = 0x0000;
	Canfilter.FilterMaskIdLow = 0x0000;
	
	Canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;	//send the message to FIFO0
	Canfilter.FilterActivation = ENABLE;
	Canfilter.FilterBank = 14;  //
	Canfilter.SlaveStartFilterBank = 14;  //For single CAN instances, this parameter is meaningless
	
	//CAN1 and CAN2 have same filter
	if(HAL_CAN_ConfigFilter(&hcan1, &Canfilter) != HAL_OK)
	{
		Error_Handler();
	}
	
	// Start the CAN peripheral 
	if (HAL_CAN_Start(&hcan1) != HAL_OK)//
   {
     /* Start Error */
     Error_Handler();
   }
     
   // Activate CAN RX notification 
   if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//ѡ��fifo0�ж�
   {
     /* Notification Error */
     Error_Handler();
   }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {

    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
 
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
		
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);

  }
} 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  {
		CAN_RxHeaderTypeDef RxMsg;
      
		uint8_t Rx_BUF[8];
		RxMsg.DLC=8;
		RxMsg.IDE=CAN_ID_STD;  //��׼��ʽ
		RxMsg.RTR=CAN_RTR_DATA; //����֡
 
     if(hcan->Instance == CAN1)
     {         
			 HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMsg, Rx_BUF);//�˴���fifo0ҲҪע��
			 if(RxMsg.StdId == CAN_MASTER)  //�����������id
			 {
					 
			 }				 
		 }
 }
 
/**
  *	@brief	��CAN�߷���ĳ��ID����Ϣ���̶�ΪRTR��ʽ������֡
  *	@param	hcan:	CAN_HandleTypeDef�ṹ��ָ�룬��pTxMsg�еĲ�����ֵ
  *	@param	ID:		����֡��IDֵ
  *	@retval	1 ���ͳɹ�
  *			0 ����ʧ��
  */




