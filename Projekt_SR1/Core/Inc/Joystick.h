

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_


typedef struct
{
	uint16_t position[4];
	int sendJoy[2];

}joystick;

void joystick_Start(ADC_HandleTypeDef *hadc, joystick *obiekt)
{
	HAL_ADC_Start_DMA(hadc, obiekt->position , 4);
}

void joystick_Stop(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Stop_DMA(hadc);
}

int voltageToAngleX_L(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[0]*1.514);;



		  return value;

}
int voltageToAngleY_L(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[1]*1.514);



		  return value;

}


int voltageToAngleX_R(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[3]*1.514);;



		  return value;

}
int voltageToAngleY_R(joystick *obiekt)
{

		int  value;
		  int minC=1600;
		  int maxC=7800;
		  int minA=4;
		  int maxA=4095;
		  value = 1600+(obiekt->position[4]*1.514);



		  return value;

}

void joystick_Control(joystick *obiekt)
{


	obiekt->sendJoy[0] = voltageToAngleX_L(obiekt);
	obiekt->sendJoy[1] = voltageToAngleY_L(obiekt);
	obiekt->sendJoy[3] = voltageToAngleX_R(obiekt);
	obiekt->sendJoy[4] = voltageToAngleY_R(obiekt);
}

#endif /* INC_JOYSTICK_H_ */
