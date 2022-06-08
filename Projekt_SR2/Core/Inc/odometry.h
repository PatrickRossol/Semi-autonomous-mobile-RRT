/*
 * odometry.h
 *
 *  Created on: Jun 12, 2021
 *      Author: john
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_
#include "pid.h"
#include "main.h"

typedef struct
{
	float x;
	float y;
	float angle;

}robot_position;


/*void move(uint8_t xCoord, uint8_t  yCoord)
{
    uint8_t angleRot;

    if(yCoord > yPos)
        angleRot = 0;
    if(yCoord < yPos)
        angleRot = 180;
act nle = anle rot
    rotate(angleRot);
    drive(abs(yPos - yCoord));

    if(xCoord > xPos)
        angleRot = 270;
    if(xCoord < xPos)
        angleRot = 90;

    rotate(angleRot);
    drive(abs(xPos - xCoord));
}*/

void rotate_robot(int angle, TIM_HandleTypeDef *htim);
int distance_encoder(float distance);
void drive_robot(float distance,uint8_t direction,TIM_HandleTypeDef *htim);


#endif /* INC_ODOMETRY_H_ */
