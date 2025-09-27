#include "wheat wheel.h"
float s1_tar_x,s1_tar_y;
extern float angle_goal;
Robot robot;
/************************************************************************************************************
*正常模式麦轮
*传入：电机目标转速结构体，x速度，y速度，旋转速度
*回传：无
*************************************************************************************************************/

void Wheat_wheel_analytic_function(Robot *robot,int tar_x,int tar_y,int tar_turn)
{
  //1号
  robot->Motor1 = tar_x - tar_y + tar_turn;
  //2号
  robot->Motor2 = - tar_x + tar_y + tar_turn;
  //3号
  robot->Motor3 = tar_x + tar_y + tar_turn ;
  //4号
  robot->Motor4 = - tar_x - tar_y + tar_turn;

}
/************************************************************************************************************
*can发送结构体
*传入：电机目标电流
*回传：无
*************************************************************************************************************/
void motorset(uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4)
{
    	send[0] = motor1 >> 8;
		send[1] = motor1;
		send[2] = motor2 >> 8;
		send[3] = motor2;
		send[4] = motor3 >> 8;
		send[5] = motor3;
		send[6] = motor4 >> 8;
		send[7] = motor4;
		
}
/************************************************************************************************************
*can发送函数
*传入：要发给电机的结构体
*回传：无
*************************************************************************************************************/

void can_send_data(uint8_t send[10])
{
	HAL_CAN_AddTxMessage(&hcan1, &can_handle_structer ,send, &send_mail_box);
}

/************************************************************************************************************
*旋转模式麦轮
*传入：电机目标转速结构体，x速度，y速度，旋转速度
*回传：无
*************************************************************************************************************/
void Wheat_wheel_analytic_function_turn(Robot *robot,int tar_x,int tar_y,int tar_turn)
{	
	float ANGLE = (IMU.quaternion.yaw - angle_goal)* 3.141f / 180.0f;
	s1_tar_x = tar_x * cos(ANGLE) + tar_y * sin(ANGLE);
	s1_tar_y = -tar_x * sin(ANGLE) + tar_y * cos(ANGLE);
	Wheat_wheel_analytic_function(robot, s1_tar_x, s1_tar_y, tar_turn);
}
