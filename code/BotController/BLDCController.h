/*
 * BLDCController.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef BLDCCONTROLLER_H_
#define BLDCCONTROLLER_H_


#define ENCODER_USE_INTERRUPTS
#include <Encoder/Encoder.h>

class BLDCController {
public:
	BLDCController();
	virtual ~BLDCController() {};

	void setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin);
	void setupEncoder( int EncoderAPin, int EncoderBPin, int CPR);
	void loop( );

	void setSpeed(float speed /* [rotations per second] */, float acc /* [rotations per second^2] */);
	float getSpeed();

	void setTorque(float torqueRatio /* [0.0-1.0] */);
	float getRevolution();

	void enable(bool doit);

	void runMenu();
private:

	// PINs for Drotek L6234 EN, IN1, IN2, IN3
	int enablePin = 0;
	int input1Pin = 0;
	int input2Pin = 0;
	int input3Pin = 0;

	int encoderAPin = 0;
	int encoderBPin = 0;

	float encoderCPR = 0;

	float targetAcc = 0;				// [rev/s^2]
	float targetSpeed = 0;				// [rev/s]
	float targetTorque = 0;				// [0..1], ratio of full torque, ends up in pwm ratio

	float magneticFieldAngle = 0;		// [rad] angle of the induced magnetic field 0=1 = 2PI
	float advanceAnglePhase = 0;		// [rad] integration of angle errors (used by PI controller)
	float advanceAngleError= 0;			// [rad] current angle error (computed by PI controller, determines torque)
	float currentSpeed = 0;				// [rev/s]
	float referenceAngle = 0;			// [rad] target angle of the rotor
	float lastReferenceAngle = 0;		// [rad]

	float encoderAngle = 0;				// [rad]
	int lastEncoderPosition = 0;		// last call of encoder value

	int getPWMValue( float angle_rad);
	void getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC);
	float turnReferenceAngle();
	void setMagneticFieldAngle(float angle);
	void readEncoder();
	void sendPWMDuty();
	uint32_t lastStepTime_us = 0;

	// data of PI controller
	static float pid_k;
	static float pid_i;

	bool isEnabled = false;

	// Encoder library
	Encoder* encoder = NULL;

	// ascii menu functionality
	void printHelp();
	float menuSpeed = 0;
	int menuAcc = 500;
	float menuTorque = 0.0;
	bool menuEnable = false;
};

#endif /* BLDCCONTROLLER_H_ */
