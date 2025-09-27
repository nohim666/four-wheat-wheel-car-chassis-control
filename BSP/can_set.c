#include "can_set.h"




/************************************************************************************************************
*can滤波器函数
*传入：无
*回传：无
*************************************************************************************************************/

void can_filter_init(void)
{
	can_filter_structer.FilterActivation = CAN_FILTER_ENABLE;
	can_filter_structer.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_structer.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_structer.FilterIdHigh = 0x0000;
	can_filter_structer.FilterIdLow = 0x0000;
	can_filter_structer.FilterMaskIdHigh = 0x0000;
	can_filter_structer.FilterMaskIdLow = 0x0000;
	can_filter_structer.FilterBank = 0;
	can_filter_structer.SlaveStartFilterBank = 0;
	can_filter_structer.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	HAL_CAN_ConfigFilter(&hcan1, &can_filter_structer);

	
	can_handle_structer.StdId = 0x200;/*标准针id，对应电机id*/
	can_handle_structer.DLC = 8;
	can_handle_structer.IDE = CAN_ID_STD;
	can_handle_structer.RTR = CAN_RTR_DATA;
	can_handle_structer.TransmitGlobalTime = DISABLE;



	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

