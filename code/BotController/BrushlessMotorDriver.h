/*
 * BLDCController.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef BLDCCONTROLLER_H_
#define BLDCCONTROLLER_H_

#include <MenuController.h>
#include <PIDController.h>
#include <Filter/ComplementaryFilter.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder/Encoder.h>

const float MaxWheelAcceleration = 2000.0; 			// [rev/s^2]
const float GearBoxRatio = 18.0/54.0*18.0/54.0; 	// two timing belts with 54/18*54/18 pulleys = 1:9

class BrushlessMotorDriver : virtual public Menuable {
public:

	BrushlessMotorDriver();
	virtual ~BrushlessMotorDriver() {};

	void setup(int motorId, MenuController* menuCtrl, bool reverse);
	void setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin);
	void setupEncoder( int EncoderAPin, int EncoderBPin, int CPR);

	void reset();

	// engine loop, returns true, if engine did something
	bool loop( );

	// set speed of motor
	void setMotorSpeed(float speed /* [rotations per second] */, float acc = MaxWheelAcceleration /* [rotations per second^2] */);
	float getMotorSpeed();
	float getIntegratedMotorAngle();

	// set speed of wheel including the gear box
	void setSpeed(float speed /* [rotations per second] */, float acc = MaxWheelAcceleration /* [rotations per second^2] */);
	float getSpeed();
	float getIntegratedAngle();

	void enable(bool doit);
	bool isEnabled() { return enabled; };

	virtual void printHelp();
	virtual void menuLoop(char ch, bool continously);
private:
	void resetEncoder();

	// PINs for Drotek L6234 EN, IN1, IN2, IN3
	int motorNo = 0;
	int enablePin = 0;
	int input1Pin = 0;
	int input2Pin = 0;
	int input3Pin = 0;

	// Encoder attached to the motor's axis
	int encoderAPin = 0;
	int encoderBPin = 0;

	// to be configured in setup
	float encoderCPR = 0;					// [] cycles per revolution. cpr times 4 gives counts per revolution

	float targetAcc = 0;					// [rev/s^2]
	float targetMotorSpeed = 0;				// [rev/s]

	float magneticFieldAngle = 0;			// [rad] angle of the induced magnetic field 0=1 = 2PI
	float advanceAngle = 0;
	float currentReferenceMotorSpeed = 0;	// [rev/s] current speed, ramp function towards targetSpeed
	float referenceAngle = 0;				// [rad] the angle the motor should have (input of PID controller)
	float lastReferenceAngle = 0;			// [rad] reference angle of last call
	float encoderAngle = 0;					// [rad] current measured angle coming from encoder
	int lastEncoderPosition = 0;			// last call of encoder value
	uint32_t lastTurnTime_us = 0;			// [us] last time we turned the reference angle
	float measuredMotorSpeed = 0;			// [rev/s] speed as given by encoder
	SpeedGainPIDController pid;

	int getPWMValue( float torque, float angle_rad);
	float turnReferenceAngle();
	void readEncoder();
	void sendPWMDuty(float torque);

	bool enabled = false;
	bool reverse = false;

	// Encoder library
	Encoder* encoder = NULL;

	// ascii menu functionality
	float menuSpeed = 0;
	int menuAcc = MaxWheelAcceleration;
	float menuTorque = 0.0;
	bool menuEnable = false;
	uint32_t lastLoopCall_ms = 0;
	PIDController pid_setup;
	LowPassFilter speedFilter;
};

#endif /* BLDCCONTROLLER_H_ */
