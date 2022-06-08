#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx.h"
#include "InverseKinematics.h"

void servo_Init(TIM_HandleTypeDef *htim);
void moveManipulator(TIM_HandleTypeDef *htim,float T1,float T2,float T3);
//void servo_Write(TIM_HandleTypeDef *htim,int CHANNEL, int angle);
void moveServo1(TIM_HandleTypeDef *htim,float angle);
void moveServo2(TIM_HandleTypeDef *htim,float  angle);
void moveServo3(TIM_HandleTypeDef *htim,float angle);
#endif /* INC_SERVO_H_ */
