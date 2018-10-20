/*
 * BrushedMotorDriver.h
 *
 *  Created on: 08.09.2018
 *      Author: JochenAlt
 */

#ifndef BRUSHEDMOTORDRIVER_H_
#define BRUSHEDMOTORDRIVER_H_


#include <MenuController.h>
#include <PIDController.h>
#include <Filter/FIRFilter.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder/Encoder.h>



class BrushedMotorDriver : virtual public Menuable {
public:
	const float MaxAcceleration = 1000.0; // [rev/s^2]
	const float MaxSpeed = 2250.0; // [rev/s]
	const float GearBoxRatio = 1.0/20.4;


	BrushedMotorDriver() {};
	virtual ~BrushedMotorDriver() {};

	void setup(MenuController* menuCtrl);
	void setupMotor( int EnablePin, int In1Pin, int In2Pin,int currentSensePin);
	void setupEncoder( int EncoderAPin, int EncoderBPin, int CPR);
	void enable(bool doIt);

	// set motor speed in [rec/s]
	float getMotorSpeed();
	void setMotorSpeed(float speed);

	float getMotorAngle();
	float getCurrentCurrent();
	void loop();

	virtual void printHelp();
	virtual void menuLoop(char ch);
private:
	void setMotorPower(float speed);

	void readEncoder();
	void readCurrentSense();
	// Encoder attached to the motor's axis
	int encoderAPin = 0;
	int encoderBPin = 0;
	int enablePin = 0;
	int CPR = 0;
	int in1Pin= 0;
	int in2Pin= 0;
	int currentSensePin = 0;
	float currentCurrent = 0;
	float encoderAngle = 0;					// [rad] current measured angle coming from encoder
	float referenceAngle = 0;				// [rad] the angle the motor should have (input of PID controller)
	int lastEncoderPosition = 0;			// last call of encoder value
	uint32_t lastLoopCall_ms = 0;			// [ms] last time loop has been called
	float referenceSpeed = 0;				// [rev/s] set speed
	bool enabled = false;
	float currentMotorPower = 0;
	PIDController pid;
	// optical Encoder
	Encoder* encoder = NULL;

	float menuSpeed = 0;
	bool menuEnable = false;
	bool logValues = false;

};

#endif /* BRUSHEDMOTORDRIVER_H_ */
