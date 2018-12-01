#ifndef SETUP_H_
#define SETUP_H_


#include <math.h>
#include <Arduino.h>

// --- general constants ---
const float OneMicrosecond_s = 0.000001;
const float Gravity = 9.81;											// [m/s^2]
const float Gravity_mm = Gravity*1000.0;							// [mm/s^2]

// --- mechanical constants ---
const float BallWeight = 0.1;										// [kg]
// const float MotorWeight = 0.195;									// [kg]
// const float EnclosureWeight = 0.2;									// [kg]
// const float BotWeight = 3*MotorWeight + EnclosureWeight;			// [kg]
const float WheelRadius = 0.035;									// [m]
const float BallRadius = 0.090;										// [m]
const float CentreOfGravityHeight = 0.150; 							// [m] center of gravity height from ground
const float MaxBotSpeed = 1.8; 										// [m/s] max speed of bot
const float MaxBotOmega= 6.0; 										// [rad/s] max vertical turn speed of bot
const float MaxBotOmegaAccel= 0.1; 									// [rad/s^2] max omega aceleration of bot
const float MaxBotAccelAccel= 0.1;							 		// [m/s^3] max acceleration acceleration of bot
const float MaxBotAccel= 1.0;							 		    // [m/s^2] max acceleration of bot

const float MaxTiltAngle = atan(MaxBotAccel/Gravity); 				// [rad] max tilt angle, 6°

// --- Teensy ---
#define LED_PIN 13					// blinking LED on Teensy

// -- power relay ---
#define POWER_RELAY_PIN 0 			// HIGH turns on power to the motors

// --- IMU ---
// possible values of sample frequency depend on IMU MP9150 are 1000/n with n=0..32,
// i.e. 90Hz, 100Hz, 111Hz, 125Hz, 142Hz, 166 Hz, 200Hz, 250Hz, 333Hz
// cpu-wise, Teensy 3.5 is capable of going up to 250 Hz, but apparently this does not give an advantage
const int SampleFrequency 					= 200; 					// [Hz] loop time as imposed by IMU frequency
const float SamplingTime 					= 1.0/SampleFrequency; 	// [s] sampling time of the general loop
#define IMU_INTERRUPT_PIN 20										// pin that listens to interrupts coming from IMU when a new measurement is in da house
#define IMU_I2C_ADDRESS 0x69										// default MPU9050 i2c address


// ---  Brushless motors   ---
const int pwmResolutionBits = 10;

// timing of wave form in brushless motors is measured in [ms], so max frequency to recompute PWM wave is 1000Hz
const int MaxBrushlessDriverFrequency = 1000;

// Teensy PWM Pins that drive input lines of Drotek L6234 breakout
const int BrushlessDriverPWMPins[3][3] = {  { 2,    3,   4},   // motor 1, PWM1, PWM2, PWM3
											{ 5,    6,   7},   // motor 2, PWM1, PWM2, PWM3
											{ 8,    9,   10}}; // motor 3, PWM1, PWM2, PWM3

// all L6234 are connected to one common enable pin
#define BRUSHLESS_DRIVER_ENABLE_PIN  24

// Magnetic Encoder AS4057D are connected via one SPI bus with three separate CS lines
const uint16_t MISO_PIN = 12;
const uint16_t MOSI_PIN = 11;
const uint16_t SCK_PIN = 27;
const uint16_t SS_PIN[3] = { 26, 28,25};

#endif /* SETUP_H_ */
