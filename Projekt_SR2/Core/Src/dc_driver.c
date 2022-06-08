/*
 * dc_driver.c
 *
 *  Created on: 8 cze 2021
 *      Author: john
 */

#include "stm32f4xx.h"
#include "pid.h"

void moveStop()
{

	//__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,PWM_CONTROL);
	//prawo góra forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

	//prawo dół forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);

	//lewo góra forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

	//lewo dół forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
}


void moveForward()
{

	//__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,PWM_CONTROL);
	//prawo góra forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

	//prawo dół forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);

	//lewo góra forward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

	//lewo dół forward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
}


void moveBackward()
{

	//__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,PWM_CONTROL);
	//prawo góra backward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

	//prawo dół backward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

	//lewo góra backward
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

	//lewo dół backward
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
}


void turnLeft()
{
	//__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,PWM_CONTROL);
	//prawo góra forward
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

		//prawo dół forward
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);

			//lewo góra backward
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

				//lewo dół backward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
}

void turnRight()//TIM_HandleTypeDef *htim,int PWM_CONTROL
{
	//__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,PWM_CONTROL);

	//prawo góra backward
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

		//prawo dół backward
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

			//lewo góra forward
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

				//lewo dół forward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
}

