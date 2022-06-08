#include "Servo.h"
#include "stm32f4xx.h"

int previous = 1600;
int previous1 = 1600;

void servo_Init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,7800);
	HAL_Delay(1000);
	 __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,1600);
	 __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,1600);


}

void moveServo1(TIM_HandleTypeDef *htim,float angle)
{

	if(previous<angle)
	{

	for (; previous < angle; previous+=10) {
		if(previous>angle)
		{
			previous = angle;
		}
	  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, previous);
	 HAL_Delay(20);
	}
	}

	else
	{

		for (; previous > angle; previous -= 10) {
			if (previous < angle) {
				previous = angle;
			}
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, previous);
			HAL_Delay(20);//60
		}
	}
}

void moveServo2(TIM_HandleTypeDef *htim,float angle)
{
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, Theta2(90));

	if(previous1<angle)
	{
	for (; previous1 < angle; previous1+=15) {
		if(previous1>angle)
		{
			previous1 = angle;
		}
	  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, previous1);
	 HAL_Delay(20);//40
	}
	}

	else
	{
		for (; previous1 > angle; previous1 -= 15) {
			if (previous1 < angle) {
				previous1 = angle;
			}
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, previous1);
			HAL_Delay(20);
		}
	}
}

void moveServo3(TIM_HandleTypeDef *htim,float angle)
{
	//angle to PWM value
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, angle);
}

void moveManipulator(TIM_HandleTypeDef *htim,float T1,float T2,float T3)
{
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,7800);
	if((previous1-1600)<(7800-previous1))
	moveServo2(htim, 1600);
	else
		{
		moveServo2(htim, 7800);
		}
	//__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,7800);
	moveServo1(htim, T1);
	moveServo2(htim, T2);
	moveServo3(htim, T3);
}
