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

const float MaxWheelAcceleration = 100.0; 			// [rev/s^2]
const float GearBoxRatio = 18.0/54.0*18.0/54.0; 	// two timing belts with 54/18*54/18 pulleys = 1:9

class MotorConfig {
public:
	void initDefaultValues();
	void print();

	// PID values for control at 0 rev/s
	PIDControllerConfig pid_position;
	PIDControllerConfig pid_speed;
	PIDControllerConfig pid_lifter;

	// mounting property: differing angle between rotor and encoder. Needs to be calibrated after assembly
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
	bool loop( );

	// set speed of motor
	void setMotorSpeed(float speed /* [rotations per second] */, float acc = MaxWheelAcceleration /* [rotations per second^2] */);
	float getMotorSpeed();
	float getIntegratedMotorAngle();
	float getSensorAngle() { return magEncoder.getSensorRead(); };

	// set speed of wheel including the gear box
	void setSpeed(float speed /* [rotations per second] */, float acc = MaxWheelAcceleration /* [rotations per second^2] */);
	float getSpeed();
	float getIntegratedAngle();

	void enable(bool doit);
	bool isEnabled() { return enabled; };

	virtual void printHelp();
	virtual void menuLoop(char ch, bool continously);
private:

	// PINs for Drotek L6234 EN, IN1, IN2, IN3
	int motorNo = 0;
	int enablePin = 0;
	int input1Pin = 0;
	int input2Pin = 0;
	int input3Pin = 0;

	float targetAcc = 0;					// [rev/s^2]
	float targetMotorSpeed = 0;				// [rev/s]

	float magneticFieldAngle = 0;			// [rad] angle of the induced magnetic field 0=1 = 2PI
	float currentReferenceMotorSpeed = 0;	// [rev/s] current speed, ramp function towards targetSpeed
	float referenceAngle = 0;				// [rad] the angle the motor should have (input of PID controller)
	float lastReferenceAngle = 0;			// [rad] reference angle of last call
	float measuredMotorSpeed = 0;			// [rev/s] speed as given by encoder
	SpeedGainPIDController pid;

	int getPWMValue( float torque, float angle_rad);
	void turnReferenceAngle(float dT);

	// return absolute angle of rotor's position relative to
	// motor's winding A (that's normalized during calibration)
	float getEncoderAngle();

	// read new value from magnetic encoder
	void readEncoderAngle();

	void sendPWMDuty(float torque);

	bool enabled = false;
	bool reverse = false;

	// Encoder library
	AS5047D magEncoder;

	// ascii menu functionality
	float menuSpeed = 0;
	int menuAcc = MaxWheelAcceleration;
	float menuTorque = 0.0;
	bool menuEnable = false;
	uint32_t lastLoopCall_ms = 0;
	TimePassedBy logTime;
};

#endif /* BLDCCONTROLLER_H_ */
