/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <Joystick.h>

#include "matrix_lib.h"
#include "Joystick.h"
#include "MY_FLASH.h"
#include "drawMenu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/*move(uint8_t xCoord, uint8_t  yCoord)
{
	uint8_t angleRot;

	if(yCoord > yPos)
		angleRot = 0;
	if(yCoord < yPos)
		angleRot = 180;

	rotate(angleRot);
	drive(abs(yPos - yCoord));

	if(xCoord > xPos)
		angleRot = 270;
	if(xCoord < xPos)
		angleRot = 90;

	rotate(angleRot);
	drive(abs(xPos - xCoord));
}

drive(uint8_t distance)
{
	while(done <= distance)
	{

	}

}


*/







/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Received[21];
uint8_t testinkr = 0;
DMA_HandleTypeDef hdma_spi1_tx;
joystick joy;
int y_rect = 4, flagaTIM3;
int rectMode, toggle;
char buffer[100];//!!50
char buffer_test[50];
uint8_t Data[3];
joystick joy;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
	return len;
}

void parse_ReceivedData()
{
	uint8_t id;
	int x = 0;
	char delim[] = ",";

	char *ptr = strtok(Received, delim);
	id = *ptr;

	if (id == 'c')
	{
		while (ptr != NULL)
		{
			ptr = strtok(NULL, delim);
			Data[x++] = atof(ptr);
		}

		for(int i=0;i<50;i++) Received[i] = NULL;
	}
}

void manualMode()
{
	uint8_t dirL, dirR;
	rectMode = 0;
	drawManualMode();
	 y_rect = 19;

	while(matrix_ReadKey() != 4)
	{
		if(matrix_ReadKey()== 3)
			y_rect = 19;

		if(matrix_ReadKey()==7)
			y_rect = 30;

		joystick_Control(&joy);

		if(y_rect == 30)
		{
			if(flagaTIM3)
			{
				sprintf(buffer,"r,%d,%d,%d,000000000\r", joy.sendJoy[0], joy.sendJoy[1], joy.sendJoy[3]);
				HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 10);

				flagaTIM3 = 0 ;
			}
		}

		if(y_rect == 19)
		{
			if(flagaTIM3)
			{
				if(joy.sendJoy[1] < 4300) dirL = 1;
				else if(joy.sendJoy[1] > 4900) dirL = 2;
				else dirL = 0;

				if(joy.sendJoy[3] < 4300) dirR = 2;
				else if(joy.sendJoy[3] > 4900) dirR = 1;
				else dirR = 0;


				sprintf(buffer,"t,%i,%i,00000000000000000000\r", dirL, dirR);
				HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 10);

				flagaTIM3 = 0 ;
			}
		}
	}
	drawMenu();
	rectMode = 0;
}

void seqMode()
{
	while(matrix_ReadKey()!= 4)
	{
		rectMode = 2;
		drawSeqMode(0);

		y_rect = 16;
		int chooseSeqMode;
		int seqCnt = 1;
		int seqNum;

		while(matrix_ReadKey()!= 4)
		{
			uint8_t coordX[6] = "";
			uint8_t coordY[6] = "";
			uint8_t coordZ[6] = "";

			int xRectCoord = 16;
			int yRectCoord = 16;
			int i = 0;
			int save = 0;

			if(matrix_ReadKey()== 3)
				 y_rect = 16;

			if(matrix_ReadKey()==7)
				 y_rect = 27;

			if (matrix_ReadKey() == 16)
			{
				if (y_rect == 16)
				{
					chooseSeqMode = 1;
				}

				else if (y_rect == 27)
				{
					chooseSeqMode = 2;
				}
			}

			if(chooseSeqMode == 1)
			{
				drawSeqMode(1);
				rectMode = 3;

				ssd1331_display_num(2, 37, seqCnt, 1, FONT_1206, BLUE);

				//X Coord
				while(matrix_ReadKey()!= 4)
				{
					if(i<6)
					{
						if(matrix_ReadKey() == 13) {coordX[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 3)  {coordX[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 7)  {coordX[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 11) {coordX[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 2)  {coordX[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 6)  {coordX[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 10) {coordX[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 1)  {coordX[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 5)  {coordX[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 9)  {coordX[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 14) {coordX[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 15) {coordX[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					}

					if(matrix_ReadKey() == 8){i--; coordX[i] = ' '; ssd1331_fill_rect(16,15,36,12,BLACK);}

					xRectCoord = i*6 + 16;
					ssd1331_display_string(16, 15, coordX, FONT_1206, GREEN);

					if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

					if(matrix_ReadKey() == 12)
					{
						if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

						for(;i< 6 ; i++) {coordX[i] = '0';}

						i = 0;
						break;
					}
				}

				//Y Coord
				xRectCoord = 64,
				yRectCoord = 16;

				while(matrix_ReadKey()!= 4)
				{
					if(i<6)
					{
						if(matrix_ReadKey() == 13) {coordY[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 3)  {coordY[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 7)  {coordY[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 11) {coordY[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 2)  {coordY[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 6)  {coordY[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 10) {coordY[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 1)  {coordY[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 5)  {coordY[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 9)  {coordY[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 14) {coordY[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 15) {coordY[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					}

					if(matrix_ReadKey() == 8){i--; coordY[i] = ' '; ssd1331_fill_rect(64,15,36,12,BLACK);}

					xRectCoord = i*6 + 64;
					ssd1331_display_string(64, 15, coordY, FONT_1206, GREEN);

					if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

					if(matrix_ReadKey() == 12)
					{
						if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

						for(;i< 5 ; i++) {coordY[i] = '0';}

						i = 0;
						break;
					}
				}

				//Z Coord
				xRectCoord = 42;
				yRectCoord = 27;

				while(matrix_ReadKey()!= 4)
				{
					if(i<6)
					{
						if(matrix_ReadKey() == 13) {coordZ[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 3)  {coordZ[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 7)  {coordZ[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 11) {coordZ[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 2)  {coordZ[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 6)  {coordZ[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 10) {coordZ[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 1)  {coordZ[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 5)  {coordZ[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 9)  {coordZ[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 14) {coordZ[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
						if(matrix_ReadKey() == 15) {coordZ[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					}

					if(matrix_ReadKey() == 8){	i--; coordZ[i] = ' '; ssd1331_fill_rect(42,27,36,12,BLACK); }

					xRectCoord = i*6 + 42;

					ssd1331_display_string(42, 27, coordZ, FONT_1206, GREEN);
					if (i <= 6) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

					//NEXT
					if(matrix_ReadKey() == 12)
					{
						ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

						for(;i< 6 ; i++) {coordZ[i] = '0';}

						sprintf(buffer_test,"n,%s,%s,%s,00000", coordX, coordY, coordZ);
						HAL_UART_Transmit(&huart2, buffer_test, strlen(buffer), 10);

						ssd1331_fill_rect(16,15,36,12,BLACK);
						ssd1331_fill_rect(64,15,36,12,BLACK);
						ssd1331_fill_rect(42,27,36,12,BLACK);
						ssd1331_fill_rect(1,39,7,9,BLACK);

						seqCnt++;

						break;
					}

					//Save
					if(matrix_ReadKey() == 16)
					{
						save = 1;

						for(;i< 6 ; i++) {coordZ[i] = '0';}

						sprintf(buffer,"n,%s,%s,%s,%i,00\r", coordX, coordY,coordZ, seqCnt);
						HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 10);

					/*	for(int i = 0; i <6 ; 1++)
						{
							coordX[i] = flashData[i];
							coordY[i] = flashData[6+i];
							coordZ[i] = flashData[12+i];
						}

						float x = stoi(coordX);
						*/

						break;
					}
				}
				if(save) break;
			}


			if(chooseSeqMode == 2)
			{
				drawSeqMode(2);
				y_rect = 16;

				while(matrix_ReadKey()!= 4)
				{
					if(matrix_ReadKey()== 3)
						 y_rect = 16;

					if(matrix_ReadKey()==7)
						 y_rect = 27;

					if(matrix_ReadKey()==11)
						 y_rect = 38;

					if (matrix_ReadKey() == 16)
					{
						if (y_rect == 16)
						{
							sprintf(buffer,"p,1,0000000000000000000000");
							HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 10);
						}

						else if (y_rect == 27)
						{
							sprintf(buffer,"p,2,0000000000000000000000");
							HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 10);
						}

						else if (y_rect == 38)
						{
							sprintf(buffer,"p,3,0000000000000000000000");
							HAL_UART_Transmit(&huart2, buffer, strlen((char*) buffer), 10);
						}
					}
				}
			}
		}
	}
	drawMenu();
}

void coordMode()
{
	rectMode = 2;
	drawCoordMode(0);

	y_rect = 16;
	int chooseSeqMode = 0;

	while(matrix_ReadKey()!= 4)
	{
		uint8_t coordX[6] = "";
		uint8_t coordY[6] = "";
		uint8_t coordZ[6] = "";

		int xRectCoord = 16;
		int yRectCoord = 16;
		int i = 0;
		int new = 0;

		if(matrix_ReadKey()== 3)
			y_rect = 16;

		if(matrix_ReadKey()==7)
			y_rect = 27;

		if (matrix_ReadKey() == 16)
		{
			if (y_rect == 16)
			{
				chooseSeqMode = 1;
			}

			else if (y_rect == 27)
			{
				chooseSeqMode = 2;
			}
		}

		if(chooseSeqMode != 0)
		{
			drawCoordMode(1);
			rectMode = 3;

			//X Coord
			while(matrix_ReadKey()!= 4)
			{
				if(i<6)
				{
					if(matrix_ReadKey() == 13) {coordX[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 3)  {coordX[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 7)  {coordX[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 11) {coordX[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 2)  {coordX[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 6)  {coordX[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 10) {coordX[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 1)  {coordX[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 5)  {coordX[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 9)  {coordX[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 14) {coordX[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 15) {coordX[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
				}

				if(matrix_ReadKey() == 8){i--; coordX[i] = ' '; ssd1331_fill_rect(16,15,36,12,BLACK);}

				xRectCoord = i*6 + 16;
				ssd1331_display_string(16, 15, coordX, FONT_1206, GREEN);

				if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

				if(matrix_ReadKey() == 12)
				{
					if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

					for(;i< 5 ; i++) {coordX[i] = '0';}

					i = 0;
					break;
				}

				if(matrix_ReadKey() == 16)
				{
					new = 1;
					break;
				}
			}

			//Y Coord
			xRectCoord = 64,
			yRectCoord = 16;

			while(matrix_ReadKey()!= 4)
			{
				if(i<6)
				{
					if(matrix_ReadKey() == 13) {coordY[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 3)  {coordY[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 7)  {coordY[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 11) {coordY[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 2)  {coordY[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 6)  {coordY[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 10) {coordY[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 1)  {coordY[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 5)  {coordY[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 9)  {coordY[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 14) {coordY[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 15) {coordY[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
				}

				if(matrix_ReadKey() == 8)  {i--; coordY[i] = ' '; ssd1331_fill_rect(64,15,36,12,BLACK); }

				xRectCoord = i*6 + 64;

				ssd1331_display_string(64, 15, coordY, FONT_1206, GREEN);
				if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

				if(matrix_ReadKey() == 12)
				{
					if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

					for(;i< 5 ; i++) {coordY[i] = '0';}

					i = 0;
					break;
				}

				if(matrix_ReadKey() == 16 || new)
				{
					new = 1;
					break;
				}
			}

			//Z Coord
			xRectCoord = 42;
			yRectCoord = 27;

			while(matrix_ReadKey()!= 4)
			{
				if(i<6)
				{
					if(matrix_ReadKey() == 13) {coordZ[i] = '.'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 3)  {coordZ[i] = '1'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 7)  {coordZ[i] = '2'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 11) {coordZ[i] = '3'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 2)  {coordZ[i] = '4'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 6)  {coordZ[i] = '5'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 10) {coordZ[i] = '6'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 1)  {coordZ[i] = '7'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 5)  {coordZ[i] = '8'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 9)  {coordZ[i] = '9'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 14) {coordZ[i] = '0'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
					if(matrix_ReadKey() == 15) {coordZ[i] = '-'; i++; ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);}
				}

				if(matrix_ReadKey() == 8){	i--; coordZ[i] = ' '; ssd1331_fill_rect(42,27,36,12,BLACK); }

				xRectCoord = i*6 + 42;

				ssd1331_display_string(42, 27, coordZ, FONT_1206, GREEN);
				if (i <= 5) ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,WHITE);

				//NEXT
				if(matrix_ReadKey() == 12)
				{
					ssd1331_fill_rect(xRectCoord, yRectCoord,6,10,BLACK);

					for(;i< 5 ; i++) {coordZ[i] = '0';}


					if(chooseSeqMode == 1)
						//sprintf(buffer,"c,%s,%s,%s,0000", coordX, coordY,coordZ);
						sprintf(buffer,"c,%s,%s,%s,0000000",coordX,coordY, coordZ);
					if(chooseSeqMode == 2)
						sprintf(buffer,"w,%s,%s,%s,0000000", coordX, coordY,coordZ);
					HAL_UART_Transmit(&huart2, buffer, strlen((char*) buffer), 10);
				}

				//NEW
				if(matrix_ReadKey() == 16 || new)
				{
					ssd1331_fill_rect(16,15,36,12,BLACK);
					ssd1331_fill_rect(64,15,36,12,BLACK);
					ssd1331_fill_rect(42,27,42,12,BLACK);
					break;
				}
			}
		}
	}
	drawMenu();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &Received, 21);
  //HAL_TIM_Base_Start(&htim11);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  joystick_Start(&hadc1, &joy);

  ssd1331_init(); //Inicjalizacja wyświetlacza OLED
  ssd1331_clear_screen(BLACK); //Ustawianie tła wyświetlacza
  drawMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  rectMode = 0;

		if (matrix_ReadKey() == 16)
		{
			if (y_rect == 4)
				manualMode();

			else if (y_rect == 15)
				seqMode();

			else if (y_rect == 26)
				coordMode();

			y_rect = 4;
		}

	 if(matrix_ReadKey()== 3)
	 {
		 y_rect = 4;
	 }
	 if(matrix_ReadKey()==7)
	 {
		 y_rect = 15;
	 }
	 if(matrix_ReadKey()==11)
	 {
		 y_rect = 26;
	 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
			//rec = 1;
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

			if(toggle)
			{
				if(rectMode == 0 || rectMode == 2)
					ssd1331_fill_rect(0, y_rect, 4, 4, WHITE);

				toggle = 0;
			}

			else
			{
				if(rectMode == 0)
					ssd1331_fill_rect(0, 4, 4, 46, BLACK);

				if (rectMode == 2)
					ssd1331_fill_rect(0, 16,4,34,BLACK);

				toggle = 1;
			}
	}

	if(htim->Instance == TIM3)
	{
		flagaTIM3 = 1;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
