#ifndef __CAN_SET_H
#define __CAN_SET_H

#include "main.h"
#include "can.h"


void can_filter_init(void);
void can_recieve_init(void);

extern uint8_t send[10];
extern uint32_t send_mail_box;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_FilterTypeDef can_filter_structer;
extern CAN_TxHeaderTypeDef can_handle_structer;
extern uint8_t rx_data[8];
extern uint8_t send[10];
extern uint32_t send_mail_box;
#endif

