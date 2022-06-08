/*
 * InverseKinematics.h
 *
 *  Created on: Feb 27, 2021
 *      Author: john
 */

#ifndef INC_INVERSEKINEMATICS_H_
#define INC_INVERSEKINEMATICS_H_


typedef struct
{
	float servo1;
	float servo2;
	float servo3;
}servo;

typedef struct
{
	float Theta1;
	float Theta2;
	float Theta3;

	float x;
	float y;
	float z;
}manipulator_position;

float Theta1(float angle);
float Theta2(float angle);
float Theta3(float angle);
float Theta2DEGREE(float angle);
float Theta1DEGREE(float angle);
float Theta3DEGREE(float angle);
void InverseKinematics(float X, float Y,float Z,servo *obiekt, manipulator_position *position,int mode);

void ForwardKinematics(float Theta1,float Theta2,float Theta3,manipulator_position *position);




#endif /* INC_INVERSEKINEMATICS_H_ */
