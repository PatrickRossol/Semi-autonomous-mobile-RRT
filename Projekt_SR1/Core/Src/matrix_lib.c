/*
 * matrix_lib.c
 *
 *  Created on: Feb 21, 2021
 *      Author: john
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "matrix_lib.h"
#define TIME 70

const uint16_t matrixRow[4] = {Keypad_R1_PIN,Keypad_R2_PIN,Keypad_R3_PIN,Keypad_R4_PIN};

const uint16_t matrixCol[4] = {Keypad_C1_PIN,Keypad_C2_PIN,Keypad_C3_PIN,Keypad_C4_PIN};

const int matrixMap[4][4] = {{Keypad_R1C1,Keypad_R1C2,Keypad_R1C3,Keypad_R1C4},
							 {Keypad_R2C1,Keypad_R2C2,Keypad_R2C3,Keypad_R2C4},
							 {Keypad_R3C1,Keypad_R3C2,Keypad_R3C3,Keypad_R3C4},
							 {Keypad_R4C1,Keypad_R4C2,Keypad_R4C3,Keypad_R4C4}};
int lastCheck = 0;
//const uint8_t matrixMap[4][4] = {{1,2,3,4},
//							 {5,6,7,8},
//							 {9,10,11,12},
//							 {13,14,15,16}};

int matrix_ReadKey()
{
	unsigned int r,c;
	int wynik=0;
	r = 0;

	if(HAL_GetTick() - lastCheck > TIME)
					{
						lastCheck = HAL_GetTick();


					}
	while(r<4)
	{
		HAL_GPIO_WritePin(Keypad_GPIO, matrixRow[r], GPIO_PIN_SET);
		c = 0;
		while(c<4)
		{
			if(HAL_GPIO_ReadPin(Keypad_GPIO, matrixCol[c])==GPIO_PIN_SET)
			{
				for(int i=0;i<4;i++)
					HAL_GPIO_WritePin(Keypad_GPIO, matrixRow[i], GPIO_PIN_RESET);

				HAL_Delay(50);
				wynik = matrixMap[r][c];
				return wynik;

			}


			c++;
		}
		HAL_GPIO_WritePin(Keypad_GPIO, matrixRow[r], GPIO_PIN_RESET);
		r++;

	}
	return 0;

}




