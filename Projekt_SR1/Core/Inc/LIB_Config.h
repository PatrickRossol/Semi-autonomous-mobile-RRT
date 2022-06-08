/**
  ******************************************************************************
  * @file    LIB_Config.h
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief     This file provides configurations for low layer hardware libraries.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USE_LIB_CONFIG_H_
#define _USE_LIB_CONFIG_H_
//Macro Definition

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "Fonts.h"

#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_5
#define CS_GPIO_Port GPIOC
#define DC_Pin GPIO_PIN_1
#define DC_GPIO_Port GPIOB
#define RES_Pin GPIO_PIN_2
#define RES_GPIO_Port GPIOB

extern SPI_HandleTypeDef hspi1;

#define __SSD1331_RES_SET()     HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
#define __SSD1331_RES_CLR()     HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);

#define __SSD1331_DC_SET()      HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
#define __SSD1331_DC_CLR()      HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

#define __SSD1331_CS_SET()      HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
#define __SSD1331_CS_CLR()      HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

// Transmisja danych przez SPI w trybie blokujacym
#define __SSD1331_WRITE_BYTE(__DATA) HAL_SPI_Transmit(&hspi1, &__DATA, 1, 1000)

// Transmisja danych przez SPI z wykorzystaniem przerwan
//#define __SSD1331_WRITE_BYTE(__DATA) HAL_SPI_Transmit_IT(&hspi1, &__DATA, 1)

// Transmisja danych przez SPI z wykorzystaniem DMA
//#define __SSD1331_WRITE_BYTE(__DATA) HAL_SPI_Transmit_DMA(&hspi1, &__DATA, 1)


/*------------------------------------------------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


#endif

/*-------------------------------END OF FILE-------------------------------*/

