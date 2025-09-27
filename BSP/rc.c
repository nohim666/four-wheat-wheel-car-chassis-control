#include "rc.h"
#include "math.h"
#include "wheat wheel.h"


/* ----------------------- RC Channel Definition---------------------------- */ 
#define RC_CH_VALUE_MIN            ((uint16_t)364 ) 
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024) 
#define RC_CH_VALUE_MAX            ((uint16_t)1684) 
  
/* ----------------------- RC Switch Definition----------------------------- */ 
#define RC_SW_UP                   ((uint16_t)1) 
#define RC_SW_MID                  ((uint16_t)3) 
#define RC_SW_DOWN                 ((uint16_t)2)  
  

/* ----------------------- PC Key Definition-------------------------------- */ 
#define KEY_PRESSED_OFFSET_W       ((uint16_t)0x01<<0)   
#define KEY_PRESSED_OFFSET_S       ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)0x01<<2)  
#define KEY_PRESSED_OFFSET_D       ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)0x01<<7) 
#define  RC_FRAME_LENGTH           18u            
   
      
#define time 2


/* ----------------------- Internal Data ----------------------------------- */ 
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];  //double sbus rx buffer to save data 
extern float angle_goal;
rc rc_ctrl;
mouse mouse1;
key key1;
extern IMU_T  IMU;
extern PID pid_angle;
extern PID pid_speed;
extern float aim_angle;
int flag_turn = 0;
int flag_change = 0;

/****************************************************************************** 
 * @fn      RemoteDataProcess 
 *  
 * @brief   resolution rc protocol data. 
 * @pData   a point to rc receive buffer. 
 * @return  None. 
 * @note    RC_CtrlData is a global variable.you can deal with it in other place. 
 */ 
void RemoteDataProcess(uint8_t *pData) 
{ 
    rc_ctrl.rightx = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;  
    rc_ctrl.righty = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; 
    rc_ctrl.leftx = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF; 
    rc_ctrl.lefty = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF; 
     
    rc_ctrl.mode = ((pData[5] >> 4) & 0x000C) >> 2; 
    rc_ctrl.s2 = ((pData[5] >> 4) & 0x0003); 
 
    mouse1.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8); 
    mouse1.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8); 
    mouse1.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
 
    mouse1.press_l = pData[12]; 
    mouse1.press_r = pData[13]; 
  
    key1.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8); 
	
}


void send_data(uint8_t *send, int key)
{
	tar_x = rc_ctrl.leftx - 1024;/*上下660*/
	tar_y = rc_ctrl.lefty - 1024;
	
	if(key == 1)/*停止模式*/
	{
		int i;
		for(i = 0; i < 8; i++)
		{
			send[i] = 0;
		}
	}
	else if(key == 2)/*正常模式*/
	{
		if(rc_ctrl.rightx != 1024) 
		{
			angle_goal += (1024 - rc_ctrl.rightx) * 0.0008;
		}
		if(angle_goal > 180)
		{
			angle_goal = angle_goal - 360;
		}
		else if(angle_goal < -180)
		{
			angle_goal = angle_goal + 360;
		}
		int angle;
		angle = angle_goal;
		if(angle_goal - IMU.quaternion.yaw >= 180)
		{
			angle -= 360;
		}
		else if(angle_goal - IMU.quaternion.yaw <= -180)
		{
			angle += 360;
		}

		int now_angle = (int)IMU.quaternion.yaw;
		pid_general_cal(&pid_angle, angle, now_angle);
		tar_turn = pid_angle.output;
		Wheat_wheel_analytic_function_turn(&robot, tar_x, tar_y, -tar_turn);
		//Wheat_wheel_analytic_function(&robot, tar_x, tar_y, -tar_turn);




		pid_general_cal(&pid_moto1, robot.Motor1 * 6, act_motor1.speed_rpm);
		pid_general_cal(&pid_moto2, robot.Motor2 * 6, act_motor2.speed_rpm);
		pid_general_cal(&pid_moto3, robot.Motor3 * 6, act_motor3.speed_rpm);
		pid_general_cal(&pid_moto4, robot.Motor4 * 6, act_motor4.speed_rpm);
		motorset(pid_moto1.output, pid_moto2.output, pid_moto3.output, pid_moto4.output);
	}
	else if(key == 3)/*旋转模式*/
	{
		tar_turn = 1500;
		flag_change = 1;
		if(rc_ctrl.rightx != 1024) 
		{
			angle_goal += (1024 - rc_ctrl.rightx) * 0.0003;
		}
		if(angle_goal > 180)
		{
			angle_goal = angle_goal - 360;
		}
		else if(angle_goal < -180)
		{
			angle_goal = angle_goal + 360;
		}
		
		Wheat_wheel_analytic_function_turn(&robot, tar_x, tar_y, tar_turn);


		pid_general_cal(&pid_moto1, robot.Motor1 * 3, act_motor1.speed_rpm);
		pid_general_cal(&pid_moto2, robot.Motor2 * 3, act_motor2.speed_rpm);
		pid_general_cal(&pid_moto3, robot.Motor3 * 3, act_motor3.speed_rpm);
		pid_general_cal(&pid_moto4, robot.Motor4 * 3, act_motor4.speed_rpm);
		motorset(pid_moto1.output, pid_moto2.output, pid_moto3.output, pid_moto4.output);
	}


}

PID *pid_general_cal(PID *pid, int target, int actual)
{	
    error = target - actual;   
	 if (error > pid->max_error)
    {
         error = pid->max_error;
    }
    else if (error < pid->min_error)
    {
        error = pid->min_error;
    }

	pid->integral += error;
    if((pid->previous_error * error) < 0)
	{
		pid->integral = pid->integral / 2;
	}	
    float derivative = (error - pid->previous_error);
    pid->output = ((pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative));
    pid->previous_error = error;
    if (pid->output >= pid->max_output)
    {
        pid->output = pid->max_output;
    }
	else if (pid->output < pid->min_output)
	{
		pid->output = pid->min_output;
	}
    return pid;
}

void PID_Init(PID *pid,float kp, float ki, float kd,float integral_zone, float max_error, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
    pid->integral_zone = integral_zone;
    pid->max_error = max_error;
    pid->min_error = -(max_error);
    pid->max_output = max_output;
	pid->min_output = -(max_output);
}




