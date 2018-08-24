/*
 * BLDCController.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef BLDCCONTROLLER_H_
#define BLDCCONTROLLER_H_

#include <MenuController.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder/Encoder.h>


const float MaxAcceleration = 500.0; // [rev/s^2]
const float GearBoxRatio = 24.0/50.0;

class OmniWheel : virtual public Menuable {
public:

	OmniWheel();
	virtual ~OmniWheel() {};

	void setup(MenuController* menuCtrl);
	void setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin);
	void setupEncoder( int EncoderAPin, int EncoderBPin, int CPR);
	void loop( );

	// set speed of motor
	void setMotorSpeed(float speed /* [rotations per second] */, float acc = MaxAcceleration /* [rotations per second^2] */);
	float getMotorSpeed();
	float getIntegratedMotorAngle();

	// set speed of wheel including the gear box
	void setSpeed(float speed /* [rotations per second] */, float acc = MaxAcceleration /* [rotations per second^2] */);
	float getSpeed();
	float getIntegratedAngle();


	void setTorque(float torqueRatio /* [0.0-1.0] */);

	void enable(bool doit);

	virtual void printHelp();
	virtual void menuLoop(char ch);
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

	int getPWMValue( float torque, float angle_rad);
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
	float menuSpeed = 0;
	int menuAcc = MaxAcceleration;
	float menuTorque = 0.0;
	bool menuEnable = false;
	uint32_t lastCall = 0;
};

#endif /* BLDCCONTROLLER_H_ */
