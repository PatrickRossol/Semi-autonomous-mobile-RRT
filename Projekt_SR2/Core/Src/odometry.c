/*
 * odometry.c
 *
 *  Created on: Jun 12, 2021
 *      Author: john
 */
#include "odometry.h"

#define ENC_ZERO 32768
#define ROBOT_ROTATE_90  3350//3475
#define ROBOT_ROTATE_90_ 4800//4700
#define ROBOT_ROTATE_180 7125//7125
#define ROBOT_ROTATE_270 9400//9400

void rotate_robot(int angle, TIM_HandleTypeDef *htim)
 {
	 pid_init(15, 2.1,0.002);
	int pid_out = 1;
	switch (angle) {

	case 180:
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO -(ROBOT_ROTATE_180),(TIM5->CNT),1);
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);

		}
		//robot->angle +=180;

		//htim5.Instance->CNT = ENC_ZERO;
		break;

	case 90:
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO -(ROBOT_ROTATE_90),(TIM5->CNT),1);

			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);

		}
		//robot->angle += 90;
		break;

	case -90:
		//pid_init(20, 2.1,0.002);
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO +(ROBOT_ROTATE_90_),(TIM5->CNT),1);
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);
		}
		//robot->angle += 90;
		break;

	case 270:
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO -(ROBOT_ROTATE_270),(TIM5->CNT),1);
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);
		}

	default:
		break;
	}
}

int distance_encoder(float distance)
{
	int encoder_tics = (int)(distance*150);

	return encoder_tics;
}


void drive_robot(float distance,uint8_t direction,TIM_HandleTypeDef *htim)
{
	pid_init(4.3, 2.16, 0.015);
	int pid_out = 1;
	if(!direction)
	{
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO +(distance_encoder(distance)),(TIM5->CNT),0);
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);
		}
		//if(axis == 'x') robot->x = (TIM5->CNT)

	}

	else
	{
		while(pid_out!=0)
		{
			pid_out = pid_calculate(ENC_ZERO -(distance_encoder(distance)),(TIM5->CNT),0);
			__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,pid_out);
		}
	}
}
