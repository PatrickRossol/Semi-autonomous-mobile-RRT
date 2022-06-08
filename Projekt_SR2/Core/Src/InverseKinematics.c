#include "InverseKinematics.h"
#include "math.h"
#include "stdlib.h"



const float  a2 = 6; // joint length 1
//const float  a4 = 5.5;// joint length 2
const float  a4 = 5;
const float a3 = 5.25;
const float a5 = 3;
const float a1 = 4.75;


float Theta1(float angle)
{

	float  value;
	  int minC=1600;
	  int maxC=7800;
	  int minA=0;
	  int maxA=180;
	  value = ((maxC-minC)/(maxA-minA))*(angle-minA)+minC;
	  return value;
}

float Theta1DEGREE(float angle)
{

	//if(angle<=0)
		//	  angle = angle+180;

	float  value;
	  int minC=1600;
	  int maxC=7800;
	  int minA=0;
	  int maxA=180;
	  value = ((maxC-minC)/(maxA-minA))*(angle-minA)+minC;
	  return value;
}
float Theta2(float angle)
{

	float  value;
	  int minC=1600;
	  int maxC=7800;
	  int minA=-90;
	  int maxA=90;
	  value = ((maxC-minC)/(maxA-minA))*(angle-minA)+minC;

	  return value;
}

float Theta2DEGREE(float angle)
{

	float  value;
	  int minC=1600;
	  int maxC=7800;
	  int minA=-90;
	  int maxA=90;
	  value = ((maxC-minC)/(maxA-minA))*(angle-minA)+minC;

	  return value;
}


float Theta3(float angle)
{
	//angle = (angle/3.14159)*180;
	float  value;
	  int minC=1600;
	  int maxC=7600;
	  int minA=0;
	  int maxA=180;
	  value = ((maxC-minC)/(maxA-minA))*(angle-minA)+minC;

	  return value;
}



float Theta3Inverse(float position)
{

	float  value;
	  int minC=0;
	  int maxC=180;
	  int minA=0;
	  int maxA=4;
	  value = ((maxC-minC)/(maxA-minA))*(position-minA)+minC;

	  return value;
}

void InverseKinematics(float X, float Y,float Z,servo *obiekt, manipulator_position *position,int mode)//odwrotna kinematyka dla przypadku gdy pierwsze servo porusza się od 0 do 180
{
	float r,phi1,phi2,phi3, T1,T2,T3;



	  r = sqrt((X*X)+(Y*Y));
	  if(mode == 0)
	  {
	  phi1 = acos(((a4*a4)-(a2*a2)-(r*r))/(-2.0*a2*r));
	  phi2 = atan(Y/X);
	  T1 = phi2-phi1;
	  phi3 = acos(((r*r)-(a2*a2)-(a4*a4))/(-2.0*a2*a4));
	  T2 = 3.14159-phi3;

	  T1 = (T1/3.14159)*180;
	  	if(X<0) T1 = T1+180;

	  T2 = (T2/3.14159)*180;

	  T3 = Theta3Inverse(Z);

	  ForwardKinematics(T1, T2, T3, position);


		  obiekt->servo1 = T1;
		  obiekt->servo2 = T2;
		  obiekt->servo3 = T3;
	  }

	  else if(mode == 1)
	  {
		phi1 = -acos(((a4 * a4) - (a2 * a2) - (r * r)) / (-2.0 * a2 * r));
		phi2 = atan(Y / X);
		T1 = phi2 - phi1;
		phi3 = acos(((r * r) - (a2 * a2) - (a4 * a4)) / (-2.0 * a2 * a4));
		T2 = -(3.14159 - phi3);

		T1 = (T1 / 3.14159) * 180;
		if (X < 0)
			T1 = T1 + 180;

		T2 = (T2 / 3.14159) * 180;

		 ForwardKinematics(T1, T2, T3, position);

		obiekt->servo1 = T1;
		obiekt->servo2 = T2;
		obiekt->servo3 = T3;
	  }

	  position->Theta1 = T1;
	  position->Theta2 = T2;
	  position->Theta3 = T3;





}

void ForwardKinematics(float T1,float T2,float T3,manipulator_position *position)
{
	T1 = T1*M_PI/180;
	T2 = T2*M_PI/180;
	T3 = T3*M_PI/180;
	position->x =  a2*cos(T1)+a4*cos(T1+T2);
	position->y =  a2*sin(T1)+a4*sin(T1+T2);
	position->z = -T3+a1+a3-a5;
}
//void InverseKinematics(float X, float Y,servo *obiekt)//odwrotna kinematyka dla przypadku gdy pierwsze servo porusza się od -90 do 90
//{
//	float r,phi1,phi2,phi3, T1,T2;
//
//
//	  r = sqrt((X*X)+(Y*Y));
//	  phi1 = acos(((a4*a4)-(a2*a2)-(r*r))/(-2.0*a2*r));
//	  phi2 = atan(Y/X);
//	  T1 = phi2-phi1;
//	  phi3 = acos(((r*r)-(a2*a2)-(a4*a4))/(-2.0*a2*a4));
//	  T2 = 3.14159-phi3;
//	  obiekt->servo1 = T1;
//	  obiekt->servo2 = T2;
//
//}


