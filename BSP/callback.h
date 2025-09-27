#ifndef __CALLBACK_H
#define __CALLBACK_H

#include "main.h"
#include "rc.h"
#include "wheat wheel.h"
#include "can_set.h"
#include "usart.h"

extern uint8_t send[10];
extern uint32_t send_mail_box;
extern CAN_HandleTypeDef hcan1;
extern rc rc_ctrl;
extern uint8_t recievedata2[18];
extern uint8_t recievedata1[18];
extern uint8_t rx_data[8];
extern uint8_t send[10];
extern uint32_t send_mail_box;

#define get_motor_measure(ptr, data)\
{\
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);\
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);\
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);\
	(ptr)->temperate = (data)[6];\
}

#endif
