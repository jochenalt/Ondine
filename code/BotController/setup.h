#ifndef SETUP_H_
#define SETUP_H_


#include <math.h>
#define IMU_INTERRUPT_PIN 20
#define IMU_I2C_ADDRESS 0x69

const int SampleFrequency 					= 100; 					// [Hz]
const float SamplingTime 					= 1.0/SampleFrequency; 	// [s] sampling time of the general loop

const float CentreOfGravityHeight = 500; 							// [mm] center of gravity height from grounm
const float MaxBotSpeed = 1800; 									// [mm/s] max speed of bot
const float MaxBotOmega= 6.0; 										// [rad/s] max vertical turn speed of bot
const float MaxBotOmegaAccel= 0.1; 									// [rad/s^2] max omega aceleration of bot
const float MaxBotAccel= 1000.;								 		// [mm/s^2] max acceleration of bot
const float MaxBotAccelAccel= 100.;								 // [mm/s^3] max acceleration acceleration of bot

const float Gravity = 9.81;												// [m/s^2]
const float Gravity_mm = Gravity*1000.0;									// [mm/s^2]

const float MaxTiltAngle = atan(MaxBotAccel/Gravity_mm); 				// max tilt angle

// max PWM value is (1<<pwmResolution)-1
const int pwmResolution = 10;

#endif /* SETUP_H_ */
