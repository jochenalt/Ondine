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


const int pwmSinArraySize = 512;


const int pwmResolution = 10;
// SVPWM (Space Vector Wave)
float pwmSpaceVector[pwmSinArraySize];


void initializeWaves() {
	const int maxPWMValue = (1<<pwmResolution)-1;
	const float spaceVectorFactor = 1.0 + sin(M_PI/4.0 + 2.0*M_PI/3)/2.0;

	static boolean initialized = false;
	if (!initialized) {
		for (int i = 0;i<pwmSinArraySize;i++) {
			float angle = float(i)/ float(pwmSinArraySize) * (M_PI*2.0);
			float phaseA = sin(angle);
			float phaseB = sin(angle - M_PI*2.0/3.0);
			float phaseC = sin(angle + M_PI*2.0/3.0);
			float voff = (min(phaseA, min(phaseB, phaseC)) + max(phaseA, max(phaseB, phaseC)))/2.0;
			float pwmSpaceVectorValue =  ((phaseA - voff)/2.0*spaceVectorFactor + 0.5)*maxPWMValue;

			Serial1.print("i=");
			Serial1.print(i);
			Serial1.print("voff=");
			Serial1.print(voff);

			Serial1.print(" SPPWM=");
			Serial1.print(pwmSpaceVectorValue);

			pwmSpaceVector[i] = pwmSpaceVectorValue;
		}
		initialized = true;
	}
}


int BLDCController::getPWMValue(int idx) {
	return torque* pwmSpaceVector[idx % pwmSinArraySize];
}

void BLDCController::getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC) {
	pwmValueA = getPWMValue(currentWaveIndex);
	pwmValueB = getPWMValue(currentWaveIndex + pwmSinArraySize / 3);
	pwmValueC = getPWMValue(currentWaveIndex + 2*pwmSinArraySize / 3);
}

BLDCController::BLDCController() {
}

BLDCController::~BLDCController() {
}



void BLDCController::setup( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {

	// initialize waves (only once)
	initializeWaves();

	// there's only one enable pin that has a short cut to EN1, EN2, and EN3 from L6234
	enablePin = EnablePin;

	input1Pin = Input1Pin;
	input2Pin = Input2Pin;
	input3Pin = Input3Pin;

	// setup L6234 input PWM pins
	analogWriteResolution(pwmResolution);

	analogWriteFrequency(input1Pin, 58593.75); // max frequency of L6234 is 150Khz. This number comes from Teensy datasheet, it seems to be the optimal frequency.
	analogWriteFrequency(input2Pin, 58593.75);
	analogWriteFrequency(input3Pin, 58593.75);


	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);

	// enable all enable lines at once
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, HIGH);

	computeNextStep();
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
		int commutationStepLength = pwmSinArraySize / 6;
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
	speedDiff = constrain(speedDiff, -abs(targetAcc)*stepInterval, +abs(targetAcc)*stepInterval);
	currentSpeed += speedDiff;
	if (currentSpeed < 0) {
		direction = BACKWARD;
	}
	else  {
		direction = FORWARD;
	}

	// default loop: assume one wave step per loop and compute the delay in between
	waveStep = 1.0;
	float const floatPrecision = 0.00001;
	if (abs(currentSpeed) > floatPrecision)
		stepInterval =  1.0/abs(currentSpeed)/pwmSinArraySize; // if currentSpeed = 0, this gets infinite and corrected later on
	else
		stepInterval =  1.0/floatPrecision/pwmSinArraySize;

	// check if we control via high speed more or slow speed mode.
	// in slow speed more we generate pwm waves that give a sin-curve
	// which is carries out in small steps of (360/pwmSinArraySize)°.
	// Once this results in a step frequency above 1KHz (which is > 3 revolutions/s) we increase the step size
	// such that we stay at approx. 1KHz.
	const float maxWaveFrequency = 500.0;
	const float minStepInterval = 1.0/maxWaveFrequency;
	if (stepInterval < minStepInterval) {
		Serial.println("min");
		waveStep = minStepInterval/stepInterval; // >= 1 or <= -1
		stepInterval = minStepInterval;
	}

	// lower limit of step interval is 100Hz
	const float minWaveFrequency = 100.0;
	const float maxStepInterval = 1.0/minWaveFrequency;
		if (stepInterval > maxStepInterval) {
		waveStep = maxStepInterval/stepInterval;
		stepInterval = maxStepInterval;
	}

	if ((abs(lastStepInterval-stepInterval) > 0.0000001) || (abs(lastWaveStep-waveStep)> 0.000001)) {
		/*
		Serial1.print("ws=");
		Serial1.print(waveStep);
		Serial1.print(" v=");
		Serial1.print(currentSpeed);
		Serial1.print(" dv=");
		Serial1.print(speedDiff);

		Serial1.print(" dt=");
		Serial1.print(stepInterval);
		Serial1.println();
		*/
		lastStepInterval = stepInterval;
		lastWaveStep = waveStep;
	}

	// next step happens in stepInterval_us
	nextStep_us = nextStep_us + stepInterval*1000000.0;
}

// call me as often as possible
void BLDCController::loop() {

	long now_us = micros();
	if (now_us > nextStep_us) {
		int pwmValueA, pwmValueB, pwmValueC;
		getPWMValues (pwmValueA, pwmValueB, pwmValueC);
		analogWrite(input1Pin, pwmValueA);
		analogWrite(input2Pin, pwmValueB);
		analogWrite(input3Pin, pwmValueC);
		// increase wave index for next loop
		if (direction == FORWARD)
			currentWaveIndex += waveStep;
		else
			currentWaveIndex -= waveStep;

		int waveSize=pwmSinArraySize;
		while (currentWaveIndex < 0)
			currentWaveIndex += waveSize;
		while (currentWaveIndex >= waveSize)
			currentWaveIndex -= waveSize;

		// Serial1.print("idx=");
		// Serial1.print(currentWaveIndex);

		computeNextStep();
		// Serial1.println();


	}
}

void BLDCController::setTorque(float newTorque) {
	torque = newTorque;
}

void BLDCController::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetSpeed = speed;
	targetAcc = acc;

	computeNextStep();
}


void BLDCController::printHelp() {
	Serial1.println("BLDC controller");
	Serial1.println("0 - stop");
	Serial1.println("+ - inc speed");
	Serial1.println("- - dec speed");
	Serial1.println("* - inc acc");
	Serial1.println("_ - dec acc");
	Serial1.println("r - revert direction");
	Serial1.println("t - increase torque");
	Serial1.println("T - decrease torque");

	Serial1.println("ESC");
}

void BLDCController::runMenu() {
	printHelp();
	while (true) { // terminates with return
		loop();
		char ch = Serial1.read();
		bool cmd = true;
		switch (ch) {
		case '0':
			menuSpeed = 0;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case '+':
			menuSpeed++;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case '-':
			menuSpeed--;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case '*':
			menuAcc++;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case '_':
			menuAcc--;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case 'r':
			menuSpeed = -menuSpeed;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case 'T':
			menuTorque += 0.05;
			if (menuTorque > 1.0)
				menuTorque = 1.0;
			setTorque(menuTorque);
			break;
		case 't':
			menuTorque -= 0.05;
			if (menuTorque < 0.0)
				menuTorque = 0.0;
			setTorque(menuTorque);
			break;

		case 'h':
			printHelp();
			break;
		case 27:
			return;
			break;
		default:
			cmd = false;
			break;
		}
		if (cmd) {
			Serial1.print("v=");
			Serial1.print(menuSpeed);
			Serial1.print(" a=");
			Serial1.print(menuAcc);
			Serial1.print(" t=");
			Serial1.print(menuTorque);

			Serial1.println(">");
		}
	}
}

