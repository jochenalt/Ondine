/*
 * BLDCController.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef BLDCCONTROLLER_H_
#define BLDCCONTROLLER_H_


class BLDCController {
	friend void hallSensor1(void* object);
	friend void hallSensor2(void* object);
	friend void hallSensor3(void* object);

public:
	BLDCController();
	virtual ~BLDCController();

	void setup( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin);
	void setupHallSensors( int hallSensor1Pin, int hallSensor2Pin, int hallSensor3Pin);

	void loop( );

	void setSpeed(float speed /* [rotations per second] */, float acc /* [rotations per second^2] */);
	void setTorque(float torqueRatio /* [0.0-1.0] */);
	void enable(bool doit);

	enum DirectionType { FORWARD, BACKWARD };

	void runMenu();
private:
	 int enablePin = 0;
	 int input1Pin = 0;
	 int input2Pin = 0;
	 int input3Pin = 0;
	 int hallSensor1Pin = 0;
	 int hallSensor2Pin = 0;
	 int hallSensor3Pin = 0;

	 float currentWaveIndex = 0;

	 DirectionType direction = FORWARD;
	 float currentSpeed = 0;
	 float targetAcc = 0;				// [rev/s^2]
	 float targetSpeed = 0;				// [rev/s]

	 float stepInterval = 0;			// interval time to increase wave index
	 float lastStepInterval = 0.0;

	 uint32_t nextStep_us = 0;				// next time in [us] when step happens
	 uint32_t lastStep_us = 0;

	 float waveStep = 0.0;				// number of indexes the wave is incremented
	 float lastWaveStep = 0.0;
	 int commutationStep = 0;
	 int lastHallSensorValue = 0;
	 float torque = 0;
	 int getPWMValue( int idx);
	 void getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC);
	 void computeNextStep();
	 void sixStepCommutation();

	 int hallSensorValue1 = 0;
	 int hallSensorValue2 = 0;
	 int hallSensorValue3 = 0;

	 bool isEnabled = false;

	 // ascii menu functionality
	 void printHelp();
	 float menuSpeed = 0;
	 int menuAcc = 500;
	 float menuTorque = 0.0;
	 bool menuEnable = false;
};

#endif /* BLDCCONTROLLER_H_ */
