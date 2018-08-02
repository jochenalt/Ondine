/*
 * BLDCController.cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <Util.h>
#include <BLDCController.h>
#include <ClassInterrupt.h>


const int pwmSinArraySize = 360;

#define INIT_WAVE
#ifdef INIT_WAVE
	// SPWM (Sine Wave)
	const int pwmSin[pwmSinArraySize] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
	// SVPWM (Space Vector Wave)
	const int pwmSpaceVector[pwmSinArraySize] = {128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109};

#else
	int pwmSin[pwmSinArraySize];
	// SVPWM (Space Vector Wave)
	int pwmSpaceVector[360];
#endif


void initializeWaves() {
	static boolean initialized = false;
	if (!initialized) {
		for (int i = 0;i<pwmSinArraySize;i++) {
			float angle = float(i)/ float(pwmSinArraySize) * 360.0 / M_PI_2;
			float phaseA = sin(angle)*255.0;
			float phaseB = sin(angle - M_PI_2/3)*255.0;
			float phaseC = sin(angle + M_PI_2/3)*255.0;

			float voff = (min(phaseA, min(phaseB, phaseC)) + max(phaseA, max(phaseB, phaseC)))/2.0;
			int pwmSinValue = phaseA/2.0 + 127.0;
			int pwmSpaceVectorValue = (phaseA - voff)/2.0 + 128.0;
#ifdef INIT_WAVE
			char buf[80];
			if (pwmSinValue != pwmSin[i]) {
				sprintf(buf, "pwmSin[%i] = %i != %i",i, pwmSinValue, pwmSin[i]);
				fatalError(buf);
			}
			if (pwmSpaceVectorValue != pwmSpaceVector[i]) {
				sprintf(buf, "pwmSpaceVector[%i] = %i != %i",i, pwmSpaceVectorValue, pwmSpaceVector[i]);
				fatalError(buf);
			}
#else
			pwmSin[i] = pwmSinValue;
			pwmSpaceVector[i] = pwmSpaceVectorValue;
#endif
		}
		initialized = true;
	}
}

int BLDCController::getWaveSize() {
	return pwmSinArraySize;
}

int BLDCController::getPWMValue(int idx) {
	if (pwmType == SIN_WAVE)
		return pwmSin[idx % pwmSinArraySize];
	else
		return pwmSpaceVector[idx % pwmSinArraySize];
}

void BLDCController::getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC) {
	if (currentWaveIndex < 0)
		currentWaveIndex += getWaveSize();
	if (currentWaveIndex > getWaveSize())
		currentWaveIndex = 0;
	pwmValueA = getPWMValue(currentWaveIndex);
	pwmValueB = getPWMValue(currentWaveIndex + (1*getWaveSize() / 3));
	pwmValueC = getPWMValue(currentWaveIndex + (2*getWaveSize() / 3));
}

BLDCController::BLDCController() {
}

BLDCController::~BLDCController() {
}



void BLDCController::setup( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {
	// there's only one enable pin that has a short cut to EN1, EN2, and EN3 from L6234
	enablePin = EnablePin;

	input1Pin = Input1Pin;
	input2Pin = Input2Pin;
	input3Pin = Input3Pin;

	// setup L6234 input PWM pins
	analogWriteFrequency(input1Pin, 100000); // max frequency of L6234 is 150Khz
	analogWriteFrequency(input2Pin, 100000);
	analogWriteFrequency(input3Pin, 100000);

	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);

	// enable all enable lines at once
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, HIGH);
}

void hallSensor1(void* object) {
	BLDCController* c = (BLDCController*)object;
	c->hallSensorValue1 = digitalRead(c->hallSensor1Pin)==HIGH?1:0;
}

void hallSensor2(void* object) {
	BLDCController* c = (BLDCController*)object;
	c->hallSensorValue2 = digitalRead(c->hallSensor2Pin)==HIGH?1:0;
}

void hallSensor3(void* object) {
	BLDCController* c = (BLDCController*)object;
	c->hallSensorValue3 = digitalRead(c->hallSensor3Pin)==HIGH?1:0;

}

void BLDCController::setupHallSensors( int HallSensor1Pin, int HallSensor2Pin, int HallSensor3Pin) {
	hallSensor1Pin = HallSensor1Pin;
	hallSensor2Pin = HallSensor2Pin;
	hallSensor3Pin = HallSensor3Pin;

	// setup hall sensor pins with pulldown (required by L6234 datasheet)
	pinMode(hallSensor1Pin, INPUT_PULLDOWN);
	pinMode(hallSensor1Pin, INPUT_PULLDOWN);
	pinMode(hallSensor1Pin, INPUT_PULLDOWN);

	// use hall sensors with interrupt to be really in time
	attachInterruptClass(hallSensor1Pin, (void*)this, hallSensor1);
	attachInterruptClass(hallSensor2Pin, (void*)this, hallSensor2);
	attachInterruptClass(hallSensor3Pin, (void*)this, hallSensor3);
}

void BLDCController::ensureSensoredCommutation() {
	// if no hall sensors are configured, return
	if ((hallSensor1Pin == 0) && (hallSensor2Pin == 0) && (hallSensor3Pin == 0))
		return;

	int hallSensorsValue =
			hallSensorValue1
			+ hallSensorValue2 * 2
			+ hallSensorValue1 * 4;

	// do something only if we just changed the hall sensor value, so every 60°
	if (hallSensorsValue != lastHallSensorValue) {
		lastHallSensorValue = hallSensorsValue;

		// six step commutation: Map binary position to sequence step
		static int hallSensorCommutationMapping[] = { 0 /* dummy */, 2 ,4,3, 6, 5, 1, 0 /* dummy */ };
		hallSensorsStep = hallSensorCommutationMapping[hallSensorsValue] - 1; // starting at 0
		if (hallSensorsStep == -1) {
			fatalError("hall sensors gave 0 sequence mapping");
		}
		int commutationStepLength = getWaveSize() / 6;
		int toBeWaveIndex = hallSensorsStep*commutationStepLength;

		if (currentWaveIndex != toBeWaveIndex) {
			warnMsg("commutation correction");
			currentWaveIndex  = toBeWaveIndex;
		}
	}
}

void BLDCController::computeNextStep() {

	// read hall sensors to adapt the phase of the wave to the current position
	ensureSensoredCommutation();

	float speedDiff = targetSpeed - currentSpeed;
	speedDiff = constrain(speedDiff, -abs(targetAcc)*stepInterval_us/1000000.0, +abs(targetAcc)*stepInterval_us/1000000.0);
	currentSpeed += speedDiff;
	if (currentSpeed < 0) {
		direction = BACKWARD;
		waveStep = -1;
	}
	else  {
		direction = FORWARD;
		waveStep = 1;
	}

	float timePerRevolution;

	// if we dont move, set timePerRevolution to infinite
	if (currentSpeed < 0.00001) {
		timePerRevolution = 1.0/abs(0.00001);
		waveStep = 0; // we stick to the same position
	}
	else
		timePerRevolution = 1.0/abs(currentSpeed);

	float newTimePerWaveIdx = timePerRevolution/getWaveSize();
	stepInterval_us = newTimePerWaveIdx / 1000000.0;

	// high speed mode: when the wave frequency goes above 1KHz (> 3 revolution/s), take multiple steps in one loop at stay at 1KHz
	if (stepInterval_us < 1000) {
		waveStep = 1000.0/stepInterval_us;
		stepInterval_us = 1000.0;
	}

	if (stepInterval_us > 1000)

	// next step happens in stepInterval_us
	lastStepTime_us = stepInterval_us;
}

// call me as often as possible
void BLDCController::loop() {

	long now_us = micros();
	if (now_us > lastStepTime_us + stepInterval_us) {
		int pwmValueA, pwmValueB, pwmValueC;
		getPWMValues (pwmValueA, pwmValueB, pwmValueC);
		analogWrite(input1Pin, pwmValueA);
		analogWrite(input2Pin, pwmValueB);
		analogWrite(input3Pin, pwmValueC);

		// increase wave index for next loop
		currentWaveIndex += waveStep;

		computeNextStep();
	}
}

void BLDCController::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetSpeed = speed;
	targetAcc = acc;

	computeNextStep();
}

