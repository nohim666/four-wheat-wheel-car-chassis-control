#include "callback.h"

int flag = 1;
extern int flag_rc;
/************************************************************************************************************
*所有回调函数
*传入：无
*回传：无
*************************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart1)/*遥控器接收*/
	{
		flag_rc = 1;
		if(flag == 1)
		{	
			flag = 2;
			RemoteDataProcess(recievedata1); 
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, recievedata1, 36);
		}
		else if(flag == 2)
		{	
			flag = 1;
			RemoteDataProcess(recievedata2);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, recievedata2, 36);
		}
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT); 
		
	} 
}
/*************************************************************************************************************/

/*************************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	int i=0 ;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
	switch(rx_header.StdId)
	{
		case 0x201: 
			get_motor_measure(&act_motor1, rx_data);
			break;
		case 0x202: 
			get_motor_measure(&act_motor2, rx_data);
			break;
		case 0x203:
			get_motor_measure(&act_motor3, rx_data);
			break;
		case 0x204:
			get_motor_measure(&act_motor4, rx_data);
			break;
	}
	for (i=0;i<8;i++)
	{
		rx_data[i] = 0;
	}
}




