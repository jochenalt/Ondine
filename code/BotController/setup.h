#ifndef SETUP_H_
#define SETUP_H_


#include <math.h>
#define IMU_INTERRUPT_PIN 20

const int SampleFrequency 					= 100; 					// [Hz]
const float SamplingTime 					= 1.0/SampleFrequency; 	// [s] sampling time of the general loop

const float CentreOfGravityHeight = 500; 							// [mm] center of gravity height from grounm
const float MaxBotSpeed = 1800; 									// [mm/s] max speed of bot
const float MaxBotOmega= 6.0; 										// [rad/s] max omega speed of bot
const float MaxBotOmegaAccel= 0.1; 									// [rad/s^2] max omega aceleration of bot
const float MaxBotAccel= 1000.;								 		// [mm/s^2] max acceleration of bot
const float MaxBotAccelAccel= 500.;								 // [mm/s^3] max acceleration acceleration of bot

const float Gravity = 9.81*1000.0;									// [mm/s^2]
const float MaxTiltAngle = atan(MaxBotAccel/Gravity); 				// max tilt angle

#endif /* SETUP_H_ */
