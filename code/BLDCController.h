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
	enum PWMType { SIN_WAVE, SPACE_VECTOR_WAVE };
	enum DirectionType { FORWARD, BACKWARD };

private:
	 int enablePin = 0;
	 int input1Pin = 0;
	 int input2Pin = 0;
	 int input3Pin = 0;
	 int hallSensor1Pin = 0;
	 int hallSensor2Pin = 0;
	 int hallSensor3Pin = 0;

	 int currentWaveIndex = 0;
	 PWMType pwmType = SIN_WAVE;

	 DirectionType direction = FORWARD;
	 float currentSpeed = 0;
	 float targetAcc = 0;				// [rev/s^2]
	 float targetSpeed = 0;				// [rev/s]

	 long lastStepTime_us = 0;			// last time when we increased wave index
	 long stepInterval_us = 0;			// interval time to increase wave index
	 long waveStep = 0;					// number of indexes the wave is incremented

	 int hallSensorsStep = 0;
	 int lastHallSensorValue = 0;
	 int getWaveSize();
	 int getPWMValue( int idx);
	 void getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC);
	 void computeNextStep();
	 void ensureSensoredCommutation();

	 int hallSensorValue1 = 0;
	 int hallSensorValue2 = 0;
	 int hallSensorValue3 = 0;

};

#endif /* BLDCCONTROLLER_H_ */
