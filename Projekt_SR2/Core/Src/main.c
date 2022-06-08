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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 2222222222222222222222222222222222222222222222222
#include <stdio.h>

#include "InverseKinematics.h"
#include "Servo.h"
#include <stdlib.h>
#include "MY_FLASH.h"
#include "dc_driver.h"
#include "pid.h"
#include "odometry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t Received;
int cpr1,cpr2,cpr4;
float cpr3;
uint8_t rxBuf[50];
uint8_t buffer[50];

//TEST UART
unsigned char buforRx[27] = {0};
unsigned char bufRxIndex = 0;
int FLAGA = 0;

int data[3];
float Data[4];
char id = 'a';
uint8_t flaga = 0;
uint8_t TRYB;
uint8_t flaga_trybu = 0;
uint8_t move = 0;
float flashData[4];
float flashData_check[4];
uint8_t flagaUART=0;
int i = 0;
int sectIT;
//int sectPLAY;
uint8_t erase = 1;
float seqNum;
robot_position robotPositon;

//cpid_t PID;
long int ENCODER_POS;

int32_t pid_out;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_MAX 49999
#define ENC_ZERO 32768
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 50);
	return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void encoder_position(){
	int diff=0;
	int pos,pos1,pos2,pos3,pos4;

	pos1 = htim1.Instance->CNT;
	pos2 = htim3.Instance->CNT;
	pos3 = htim4.Instance->CNT;
	pos4 = htim5.Instance->CNT;

	pos = (pos1+pos2+pos3+pos4)/4;


	htim1.Instance->CNT = ENC_ZERO;
	htim3.Instance->CNT = ENC_ZERO;
	htim4.Instance->CNT = ENC_ZERO;
	htim5.Instance->CNT = ENC_ZERO;



	diff = pos - ENC_ZERO;
	ENCODER_POS += diff;
	//ENC_L.absolute_pos -= diff;
	//ENC_L.curr_speed = - diff*463/10;
}


/*void initialize_PID(int8_t kp, int8_t ki, int8_t kd){

	  pid_init(&PID, kp*0.1f, ki*0.01f, kd*0.01f, 4, 40);  // 10ms ze wzgledu na 25Hz
	  PID.p_max = pid_scale(&PID, MOTOR_MAX);
	  PID.p_min = pid_scale(&PID, -MOTOR_MAX);
	  PID.i_max = pid_scale(&PID, MOTOR_MAX);
	  PID.i_min = pid_scale(&PID, -MOTOR_MAX);
	  PID.d_max = pid_scale(&PID, MOTOR_MAX);
	  PID.d_min = pid_scale(&PID, -MOTOR_MAX);
	  PID.total_max = pid_scale(&PID, MOTOR_MAX);
	  PID.total_min = pid_scale(&PID, -MOTOR_MAX);

}*/





void parse_ReceivedData() {



		int x = 0;
		int init_size = strlen(rxBuf);
		char delim[] = ",";



		char *ptr = strtok(rxBuf, delim);
		id = *ptr;

			while (ptr != NULL) {
				//printf("'%s'\n", ptr);
				ptr = strtok(NULL, delim);
				Data[x++] = atof(ptr);
				//sscanf(ptr, "%d", &data[x++]);
			}

			for (int i = 0; i < 50; i++)
				rxBuf[i] = NULL;

	//NOWE
			/*int x = 0;
			int init_size = strlen(buforRx);
			char delim[] = ",";



			char *ptr = strtok(buforRx, delim);
			id = *ptr;

				while (ptr != NULL) {
					//printf("'%s'\n", ptr);
					ptr = strtok(NULL, delim);
					Data[x++] = atof(ptr);
					//sscanf(ptr, "%d", &data[x++]);
				}

				for (int i = 0; i < 50; i++)
					buforRx[i] = NULL;*/

}

void MOVE(float xCoord, float  yCoord, TIM_HandleTypeDef *htim)
{
    float angleRot;

    if(yCoord != robotPositon.y)
    {
        if(yCoord > robotPositon.y)
        {
            angleRot = -robotPositon.angle;    // 0
            robotPositon.angle = 0;
        }

        if(yCoord < robotPositon.y)
        {
            if(robotPositon.angle >= 0) angleRot = 180 - robotPositon.angle;  //180
            if(robotPositon.angle < 0) angleRot = -180 - robotPositon.angle;
            robotPositon.angle = 180;
        }

        rotate_robot(angleRot, htim);
        HAL_Delay(1000);
        htim5.Instance->CNT = ENC_ZERO;
        drive_robot(abs(robotPositon.y - yCoord), 0, htim);
        HAL_Delay(1000);
        htim5.Instance->CNT = ENC_ZERO;
        robotPositon.y = yCoord;
    }

    if(xCoord != robotPositon.x)
    {
        if(xCoord > robotPositon.x)
        {
            angleRot = 90 - robotPositon.angle;
            robotPositon.angle = 90;
        }
        if(xCoord < robotPositon.x)
        {
            if(robotPositon.angle == 0) angleRot = -90 - robotPositon.angle;
            if(robotPositon.angle == 180) angleRot = -90 + robotPositon.angle;
            robotPositon.angle = -90;
        }

        rotate_robot(angleRot, htim);
        HAL_Delay(1000);
        htim5.Instance->CNT = ENC_ZERO;

        drive_robot(abs(robotPositon.x - xCoord), 0, htim);
        HAL_Delay(1000);
        htim5.Instance->CNT = ENC_ZERO;
        robotPositon.x = xCoord;
    }
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  MY_FLASH_SetSectorAddrs(6, 0x08040000);

  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
  //HAL_UART_Receive_IT(&huart2, &buforRx[bufRxIndex], 1);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  servo obiekt;
  manipulator_position position;
  servo_Init(&htim2);

 // initialize_PID(3, 0, 0);
 pid_init(4.3, 2.16, 0.015);//pid_init(10, 3, 0.2) pid_init(9, 6, 0.35) pid_init(9, 4.7, 0.35
  htim1.Instance->CNT = ENC_ZERO;
  htim3.Instance->CNT = ENC_ZERO;
  htim4.Instance->CNT = ENC_ZERO;
  htim5.Instance->CNT = ENC_ZERO;
  cpr3 = 1;
  cpr1 = 0;

  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 700);
  //moveManipulator(&htim2, Theta1DEGREE(180), Theta2DEGREE(-90), Theta3(180));

  //HAL_Delay(7000);

  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1600);

  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 7800);
   //HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//  rotate_robot(180, &htim2);
	//pid_out = pid_calculate(ENC_ZERO -(1*3840),(TIM5->CNT),1);

	// __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,pid_out);
	 // rotate_robot(180, &robotPositon, &htim2);
	 //cpr1 = (TIM5->CNT);
	 //cpr2 = ENC_ZERO -(1*3840);
	 //cpr3 = (ENC_ZERO -TIM5->CNT)/150; // liczenie cm
	 // if(!cpr1)
	  //{
	//	  MOVE(50.0, 70.0, &htim2);
	//	  MOVE(-30.0,50.0,&htim2);
	//	  cpr1++;
	//  }

	  //asdasd
	 // rotate_robot(180,&htim2);
	 // MOVE(40.0,60.0,&htim2);
	  /*if(cpr1 == 0)
	  {
		 cpr1 = 1;

	  drive_robot(30.0,0,&htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);

	  rotate_robot(90, &robotPositon, &htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);
	  drive_robot(40.0,0,&htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);
	  rotate_robot(90, &robotPositon, &htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);
	  drive_robot(30.0,0,&htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);
	  rotate_robot(90, &robotPositon, &htim2);
	  htim5.Instance->CNT = ENC_ZERO;
	  HAL_Delay(1000);
	  	drive_robot(40.0,0,&htim2);
	  	  htim5.Instance->CNT = ENC_ZERO;
	  	 HAL_Delay(1000);
	  	rotate_robot(90, &robotPositon, &htim2);
	  	htim5.Instance->CNT = ENC_ZERO;
	  	 HAL_Delay(1000);



	   //HAL_Delay(200);
	  //drive_robot(47.6,0,&htim2);
	  //htim5.Instance->CNT = ENC_ZERO;
	  //HAL_Delay(200);
	  }*/
	  //cpr3 = (float)(TIM5->CNT-ENC_ZERO)/150;
	  //cpr4 = (float)(ENC_ZERO-TIM5->CNT)/150;

	  //moveForward(&htim9, 999);

	  //turnLeft(&htim2,20000);

	 /* if(FLAGA == 1)
		  {

				 parse_ReceivedData();
				 flagaUART = 0;
				 HAL_UART_Receive_IT(&huart2, &buforRx[bufRxIndex], 1);

				 if(id == 'c')
				 {
					 move = 1;
				 }

		  }

	  while(id == 't')
	  	 	  {
		  if(FLAGA == 1)
			  {
				  parse_ReceivedData();
				  FLAGA = 0 ;
				  HAL_UART_Receive_IT(&huart2, &buforRx[bufRxIndex], 1);

			  }
	  	 		  if(id == 't')
	  	 		  {
	  	 			 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,20000);

	  	 			 //if(Data[0] == 0 && Data[1] == 0) moveStop();

	  				if (Data[0] == 0) {
	  					//lewo góra forward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

	  					//lewo dół forward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	  				}

	  				else if (Data[0] == 1) {
	  					//lewo góra forward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

	  					//lewo dół forward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	  				}

	  				else if (Data[0] == 2) {
	  					//lewo góra backward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

	  					//lewo dół backward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

	  				}

	  				if (Data[1] == 0) {
	  					//prawo góra forward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

	  					//prawo dół forward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
	  				}

	  				else if (Data[1] == 1) {
	  					//prawo góra forward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

	  					//prawo dół forward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
	  				}

	  				else if (Data[1] == 2) {
	  					//prawo góra backward
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

	  					//prawo dół backward
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

	  				}




	  	 		  }

	  	 	  }*/

	  if(flagaUART == 1)
	  		  {

	  				 parse_ReceivedData();
	  				 flagaUART = 0;
	  				HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);

	  				 if(id == 'c')
	  				 {
	  					 move = 1;
	  				 }

	  		  }

	  //RECZNY MANIP
	  while(id == 'r')
	  {
		  if(flagaUART == 1)
		  {
			  parse_ReceivedData();
			  flagaUART = 0;
			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
		  }

		  if(id == 'r')
		  {
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Data[0]);
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Data[1]);
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Data[2]);

		  }
		  if(id != 'r')
		  {
			  moveManipulator(&htim2, Data[0], Data[1], Data[2]);
		  }
	  }

	  //RECZNY POJAZD
	 	  while(id == 't')
	 	  {
	 		  if(flagaUART == 1)
	 		  {
	 			  parse_ReceivedData();
	 			  flagaUART = 0;
	 			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
	 		  }

	 		  if(id == 't')
	 		  {
	 			 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,20000);

	 			 //if(Data[0] == 0 && Data[1] == 0) moveStop();

				if (Data[0] == 0) {
					//lewo góra forward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

					//lewo dół forward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
				}

				else if (Data[0] == 1) {
					//lewo góra forward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);

					//lewo dół forward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
				}

				else if (Data[0] == 2) {
					//lewo góra backward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);

					//lewo dół backward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

				}

				if (Data[1] == 0) {
					//prawo góra forward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

					//prawo dół forward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
				}

				else if (Data[1] == 1) {
					//prawo góra forward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

					//prawo dół forward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
				}

				else if (Data[1] == 2) {
					//prawo góra backward
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

					//prawo dół backward
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

				}




	 		  }

	 	  }



	  //NOWA SEKWECJA
	  while(id == 'n')
	  {
		  if(flagaUART == 1)
		  {
			  parse_ReceivedData();
			  flagaUART = 0;
			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
		  }

		  if(sectIT == 0)
		  {
			  HAL_FLASH_Unlock();
			    	//Erase the required Flash sector
			    	FLASH_Erase_Sector(6, FLASH_VOLTAGE_RANGE_3);
			    	HAL_FLASH_Lock();
		  }

		  if(id == 'n')
		  {
			  if(TRYB == 1 && flagaUART == 0)
			  {
			  if(Data[3] == 0)
			  {
				  MY_FLASH_WriteN(sectIT*16+4, Data, 3, DATA_TYPE_32);
				 // MY_FLASH_ReadN(sectIT*16+4, flashData, 3, DATA_TYPE_32);

				  //MY_FLASH_ReadN(4, flashData_check, 3, DATA_TYPE_32);
				  sectIT++;
				  flagaUART = 0;
			  }

			  if(Data[3] != 0)
			  {
				  MY_FLASH_WriteN(0, &Data[3], 1, DATA_TYPE_32);
				  MY_FLASH_WriteN(sectIT*16+4, Data, 3, DATA_TYPE_32);
				  sectIT = 0;
				  id = 'a';
			  }
			  TRYB = 0;
			  }
		  }
	  }

	  //PLAY SEKWENCJA
	 /* while(id == 'p')
	  {
		  if(flagaUART == 1)
		  {
			  parse_ReceivedData();
			  flagaUART = 0;
			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
		  }


		  if(Data[0] == 1)
		  {
			  MY_FLASH_ReadN(0, &seqNum, 1, DATA_TYPE_32);

			  for(int sectPLAY = 0;sectPLAY < (int)seqNum; sectPLAY++ )
			  {
				  MY_FLASH_ReadN(sectPLAY*16+4, flashData, 3, DATA_TYPE_32);

				  InverseKinematics(flashData[0], flashData[1], flashData[2], &obiekt, &position, 0);
				  moveManipulator(&htim2, Theta1(obiekt.servo1), Theta2(obiekt.servo2), Theta3(obiekt.servo3));

				  HAL_Delay(1000);
			  }
			  id = 'a';

		  }
	  }*/

	  while(id == 'p')
	  	  {
	  		  if(flagaUART == 1)
	  		  {
	  			  parse_ReceivedData();
	  			  flagaUART = 0;
	  			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
	  		  }


	  		  if(Data[0] == 1)
	  		  {
	  			  MY_FLASH_ReadN(0, &seqNum, 1, DATA_TYPE_32);

	  			  for(int sectPLAY = 0;sectPLAY < (int)seqNum; sectPLAY++ )
	  			  {
	  				  MY_FLASH_ReadN(sectPLAY*16+4, flashData, 3, DATA_TYPE_32);

	  				if((sectPLAY+1)%2 == 0)
	  				{
	  					InverseKinematics(flashData[0], flashData[1], flashData[2], &obiekt, &position, 0);
	  					moveManipulator(&htim2, Theta1(obiekt.servo1), Theta2(obiekt.servo2), Theta3(obiekt.servo3));
	  					HAL_Delay(2000);
	  					servo_Init(&htim2);
	  					HAL_Delay(2000);
	  				}
	  				else
	  				{
	  					MOVE(flashData[0], flashData[1], &htim2);
	  					HAL_Delay(1000);
	  				}

	  			  }
	  			  id = 'a';

	  		  }
	  	  }

	  //TRYB COORDINATES
	  while(id == 'c')
	  {
		  if(flagaUART == 1)
		  {
			  parse_ReceivedData();
			  flagaUART = 0;
			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
			  move = 1;
		  }

		  if(id == 'c')
		  {
			  if(move  == 1)
			  {
				  if(Data[0]!= 0)
				  {
					  MOVE(Data[0]-21, Data[1], &htim2);
				  }

				if(Data[0] == 0) MOVE(Data[0], Data[1]-21, &htim2);
			  // InverseKinematics(Data[0], Data[1], Data[2], &obiekt, &position, 0);
				InverseKinematics(0, 11, 0, &obiekt, &position, 0);
			   moveManipulator(&htim2, Theta1(obiekt.servo1), Theta2(obiekt.servo2), Theta3(obiekt.servo3));
			   move = 0;
			  }

		  }
	  }

	  //TRYB COORDINATES
	 	  while(id == 'w')
	 	  {
	 		  if(flagaUART == 1)
	 		  {
	 			  parse_ReceivedData();
	 			  flagaUART = 0;
	 			  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rxBuf, 26);
	 			  move = 1;
	 		  }

	 		  if(id == 'w')
	 		  {
	 			  if(move  == 1)
	 			  {
	 			   InverseKinematics(Data[0], Data[1], Data[2], &obiekt, &position, 0);
	 			   moveManipulator(&htim2, Theta1(obiekt.servo1), Theta2(obiekt.servo2), Theta3(obiekt.servo3));
	 			   move = 0;
	 			  }

	 		  }
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 23;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 7199;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 29999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		flagaUART = 1;
		TRYB = 1;

	}

	//NOWEEEEEEEEE
	/*if (huart->Instance == USART2) {

		if(buforRx[0] == 't')
		{
			if(buforRx[bufRxIndex] == 0x0D){
				FLAGA = 1;
				bufRxIndex = 0;
			} else{
				bufRxIndex++;
				HAL_UART_Receive_IT(&huart2, &buforRx[bufRxIndex], 1);
				if(bufRxIndex>27){
					bufRxIndex= 0;
				}
			}

		}
		else
			HAL_UART_Receive_IT(&huart2, &buforRx[0], 1);
	}*/

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
