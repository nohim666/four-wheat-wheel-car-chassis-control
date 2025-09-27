#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "wheat wheel.h"
#include "imu_data_analysis.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "math.h"

typedef struct
{
	int16_t	 	ecd;	
	int16_t	 	speed_rpm;
    int16_t  	given_current;
    int16_t     temperate;
	
}moto_measure_t;

typedef struct 
	{ 
		int16_t rightx; 
		int16_t righty; 
		int16_t leftx; 
		int16_t lefty; 
		uint8_t  mode; 
		uint8_t  s2; 
	}rc; 

typedef	struct 
	{ 
		int16_t x; 
		int16_t y; 
		int16_t z; 
		uint8_t press_l; 
		uint8_t press_r; 
	}mouse; 

typedef	struct 
	{ 
		uint16_t v; 
	}key; 

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;     
    int16_t output;
    float integral_zone;       
    float max_error;   
    float min_error;
    float max_output;      
	float min_output;
}PID;
extern PID pid_moto1;
extern PID pid_moto2;
extern PID pid_moto3;
extern PID pid_moto4;
PID *pid_general_cal(PID *pid, int target, int actual);
void PID_Init(PID *pid,float kp, float ki, float kd,float integral_zone, float max_error, float max_output);
void RemoteDataProcess(uint8_t *pData) ;
void send_data(uint8_t *send, int key);
extern DMA_HandleTypeDef hdma_usart1_rx;
extern int tar_x,tar_y,tar_turn;
extern IMU_T  IMU;
extern mpu_data_t mpu_data;
extern moto_measure_t act_motor1;
extern moto_measure_t act_motor2;
extern moto_measure_t act_motor3;
extern moto_measure_t act_motor4;
extern float error;
#endif
