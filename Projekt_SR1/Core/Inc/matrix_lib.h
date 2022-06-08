/*
 * matrix_lib.h
 *
 *  Created on: Feb 21, 2021
 *      Author: john
 */

#ifndef INC_MATRIX_LIB_H_
#define INC_MATRIX_LIB_H_

#define Keypad_GPIO GPIOE

#define Keypad_R1_PIN GPIO_PIN_7
#define Keypad_R2_PIN GPIO_PIN_8
#define Keypad_R3_PIN GPIO_PIN_9
#define Keypad_R4_PIN GPIO_PIN_10


#define Keypad_C1_PIN GPIO_PIN_11
#define Keypad_C2_PIN GPIO_PIN_12
#define Keypad_C3_PIN GPIO_PIN_13
#define Keypad_C4_PIN GPIO_PIN_14

//mapowanie klawiszy

#define Keypad_R1C1 1
#define Keypad_R1C2 2
#define Keypad_R1C3 3
#define Keypad_R1C4 4

#define Keypad_R2C1 5
#define Keypad_R2C2 6
#define Keypad_R2C3 7
#define Keypad_R2C4 8

#define Keypad_R3C1 9
#define Keypad_R3C2 10
#define Keypad_R3C3 11
#define Keypad_R3C4 12

#define Keypad_R4C1 13
#define Keypad_R4C2 14
#define Keypad_R4C3 15
#define Keypad_R4C4 16


int matrix_ReadKey(void);
int matrix_ReadKeyBounce();



#endif /* INC_MATRIX_LIB_H_ */
