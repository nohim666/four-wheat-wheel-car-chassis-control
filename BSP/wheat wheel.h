#ifndef __WHEAT_WHEEL_H
#define __WHEAT_WHEEL_H

#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "imu_data_analysis.h"
#include "math.h"

typedef struct
{
  int Motor1;
  int Motor2;
  int Motor3;
  int Motor4;
}Robot;

typedef struct
{
  int Motor1;
  int Motor2;
  int Motor3;
  int Motor4;
}goal;

extern Robot robot;
extern IMU_T  IMU;
extern mpu_data_t mpu_data;
extern goal speed_goal;
extern uint8_t send[10];
extern uint32_t send_mail_box;
void Wheat_wheel_analytic_function(Robot *robot,int tar_x,int tar_y,int tar_turn);
void Wheat_wheel_analytic_function_turn(Robot *robot,int tar_x,int tar_y,int tar_turn);
void motorset(uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4);
void can_send_data(uint8_t send[10]);
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_FilterTypeDef can_filter_structer;
extern CAN_TxHeaderTypeDef can_handle_structer;

#endif
