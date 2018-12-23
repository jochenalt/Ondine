/*
 * BLDCController.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef BLDCCONTROLLER_H_
#define BLDCCONTROLLER_H_

#include <libraries/MenuController.h>
#include <libraries/PIDController.h>
#include <Filter/ComplementaryFilter.h>
#include <Encoder/AS5047D.h>
#include <TimePassedBy.h>

const float GearBoxRatio = 18.0/54.0*18.0/54.0; 	// two timing belts with 54/18*54/18 pulleys = 1:9

class MotorConfig {
public:
	void initDefaultValues();
	void print();

	// PID values for control at 0 rev/s
	PIDControllerConfig pid_position;
	PIDControllerConfig pid_speed;
	PIDControllerConfig pid_lifter;

	// mounting property: differing angle between rotor and encoder.
	// Needs to be calibrated after assembly to ake into account that
	// the magnet gets a random position to the rotor
	float phaseAAngle[3];
};


class BrushlessMotorDriver : virtual public Menuable {
public:

	BrushlessMotorDriver();
	virtual ~BrushlessMotorDriver() {};

	void setup(int motorId, MenuController* menuCtrl, bool reverse);
	void setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin);
	void setupEncoder( uint8_t clientSelectPin);

	void reset();
	void calibrate();

	// engine loop, returns true, if engine did something
	bool loop(uint32_t now_us );

	// set speed of motor
	void setMotorSpeed(float speed /* [rev/s] */);
	float getMotorSpeed();
	float getIntegratedMotorAngle();

	// set speed of wheel including the gear box
	void setSpeed(float speed /* [rev/s] */);
	float getSpeed();
	float getIntegratedAngle();

	void enable(bool doit);
	bool isEnabled() { return enabled; };

	virtual void printHelp();
	virtual void menuLoop(char ch, bool continously);

	float resetAngle();
private:

	// PINs for Drotek L6234 EN, IN1, IN2, IN3
	int motorNo = 0;						// identificator of the motor 0..2
	int enablePin = 0;						// Teensy Pin that is connected to L6234 enable PIN
	int input1Pin = 0;						// Teensy Pin that is connected to L6234 IN1
	int input2Pin = 0;						// Teensy Pin that is connected to L6234 IN2
	int input3Pin = 0;						// Teensy Pin that is connected to L6234 IN3

	float targetMotorSpeed = 0;				// [rev/s]
	float magneticFieldAngle = 0;			// [rad] angle of the induced magnetic field 0=1 = 2PI
	float currentReferenceMotorSpeed = 0;	// [rev/s] current speed, ramp function towards targetSpeed
	float referenceAngle = 0;				// [rad] the angle the motor should have (input of PID controller)
	float measuredMotorSpeed = 0;			// [rev/s] speed as given by encoder
	SpeedGainPIDController pid;				// PID controller that decreases gain with speed

	// get sin wave value out of given torque and angle
	int getPWMValue( float torque, float angle_rad);

	// called by loop, turns the magnetic field along the passed time
	void turnReferenceAngle(float dT);

	// return absolute angle [rad] of rotor's position relative to
	// motor's winding A (that's normalized during calibration)
	float getEncoderAngle();

	// read new value from magnetic encoder, getEncoderAngle returns the value read
	void readEncoderAngle();

	// set PWM value (=torque) to PWM pins that are connected to L6234 driver
	void sendPWMDuty(float torque);

	bool enabled = false;
	bool reverse = false;

	// Encoder library
	AS5047D magEncoder;

	// time measurement
	TimeLoop timeLoop;

	// average speed measurement
	TimePassedBy measurementTimer;
	float measurementAngle = 0;


	// ascii menu functionality
	float menuSpeed = 0;
	bool menuEnable = false;
};

#endif /* BLDCCONTROLLER_H_ */
