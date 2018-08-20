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
#include <utilities/TimePassedBy.h>


float BLDCController::pid_k = .5;
float BLDCController::pid_i = 0.8;

const float minTorque = 0.05; // [PWM ratio]
const float maxAdvanceAngle = radians(15); // [rad]

// max PWM value is (1<<pwmResolution)-1
const int pwmResolution = 8;

// array to store pre-computed values of space vector wave form (SVPWM)
const int svpwmArraySize = 244;
int svpwmTable[svpwmArraySize];

void precomputeSVPMWave() {
	const int maxPWMValue = (1<<pwmResolution)-1;
	const float spaceVectorFactor = 1.15; // empiric to reach full pwm scale
	static boolean initialized = false;
	if (!initialized) {
		for (int i = 0;i<svpwmArraySize;i++) {
			float angle = float(i)/ float(svpwmArraySize) * (M_PI*2.0);
			float phaseA = sin(angle);
			float phaseB = sin(angle + M_PI*2.0/3.0);
			float phaseC = sin(angle + M_PI*4.0/3.0);

			// neat software trick to avoid the switch of 6 phases everyone else is doing
			float voff = (min(phaseA, min(phaseB, phaseC)) + max(phaseA, max(phaseB, phaseC)))/2.0;

			float pwmSpaceVectorValue =  ((phaseA - voff)/2.0*spaceVectorFactor + 0.5)*maxPWMValue;
			float pwmSinValue =  (phaseA/2.0 + 0.5)*maxPWMValue;

			if (pwmSpaceVectorValue<maxPWMValue/2)
				pwmSpaceVectorValue = 0;
			svpwmTable[i] =  pwmSpaceVectorValue;
			svpwmTable[i] =  pwmSinValue;

		}
		initialized = true;
	}
}

int BLDCController::getPWMValue(float angle_rad) {

	// torque is increased with advance angle error, which is the difference
	// between to-be reference angle and actual angle as indicated by the encoder
	float torque = targetTorque + (1.0-targetTorque)*(min(abs(advanceAngleError)/maxAdvanceAngle,1.0));

	// map input angle to 0..2*PI
	if (angle_rad < 0)
		angle_rad += (int)(abs(angle_rad)/(2.0*M_PI) + 1)*2.0*M_PI;
	int angleIndex = ((int)(angle_rad / ( 2.0*M_PI) * svpwmArraySize)) % svpwmArraySize;
	if ((angleIndex <0) || (angleIndex > svpwmArraySize))
		fatalError("cvpwm table look up error");

	return  torque * svpwmTable[angleIndex];
}

// return pwm values that give the magnetic field angle
void BLDCController::getPWMValues (int &pwmValueA, int &pwmValueB, int &pwmValueC) {
	pwmValueA = getPWMValue(magneticFieldAngle);
	pwmValueB = getPWMValue(magneticFieldAngle + 1.0*(2.0*M_PI)/3.0);
	pwmValueC = getPWMValue(magneticFieldAngle + 2.0*(2.0*M_PI)/3.0);
}

BLDCController::BLDCController() {
	// initialize precomputed spvm values (only once)
	precomputeSVPMWave();
}


void BLDCController::setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {
	// there's only one enable pin that has a short cut to EN1, EN2, and EN3 from L6234
	enablePin = EnablePin;

	input1Pin = Input1Pin;
	input2Pin = Input2Pin;
	input3Pin = Input3Pin;

	// setup L6234 input PWM pins
	analogWriteResolution(pwmResolution);

	// choose the frequency that it just can't be heard
	analogWriteFrequency(input1Pin, 20000);
	analogWriteFrequency(input2Pin, 20000);
	analogWriteFrequency(input3Pin, 20000);

	// has to be pwm pins
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);

	// enable all enable lines at once (Drotek L6234 board has all enable lines connected)
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, LOW); // start with disabled motor

	// initialize magnetic field values
	setMagneticFieldAngle(0);
}

void BLDCController::setupEncoder(int EncoderAPin, int EncoderBPin, int CPR) {
	encoderAPin = EncoderAPin;
	encoderBPin = EncoderBPin;

	encoderCPR = CPR;
	encoder = new Encoder(encoderAPin, encoderBPin);
}

// turn magnetic field of the motor according to current speed
void BLDCController::setMagneticFieldAngle(float angle) {
	// increase angle of magnetic field
	magneticFieldAngle = angle;
}

float BLDCController::turnReferenceAngle() {
	uint32_t now_us = micros();
	uint32_t timePassed_us = now_us - lastStepTime_us;

	// check for overflow on micros() (happens every 70 minutes at teensy's frequency)
	if (now_us < lastStepTime_us)
		timePassed_us = 4294967295 - timePassed_us;

	float timePassed_s = (float)timePassed_us/1000000.0;
	lastStepTime_us = now_us;

	// accelerate to target speed
	float speedDiff = targetSpeed - currentSpeed;
	speedDiff = constrain(speedDiff, -abs(targetAcc)*timePassed_s, +abs(targetAcc)*timePassed_s);
	currentSpeed += speedDiff;

	// compute angle difference compared to last invokation
	referenceAngle += currentSpeed * timePassed_s * 2.0 * M_PI;

	return timePassed_s;
}

void BLDCController::readEncoder() {
	if (encoder == NULL) {
		// without encoder assume perfect motor
		encoderAngle = referenceAngle;
	}
	else {
		// find encoder position and increment the encoderAngle accordingly
		int32_t encoderPosition= encoder->read();
		encoderAngle += ((float)(lastEncoderPosition - encoderPosition))/(float)encoderCPR*2.0*M_PI/4.0;
		lastEncoderPosition = encoderPosition;
	}
}

// set the pwm values matching the current magnetic field angle
void BLDCController::sendPWMDuty() {
	int pwmValueA, pwmValueB, pwmValueC;
	getPWMValues (pwmValueA, pwmValueB, pwmValueC);
	analogWrite(input1Pin, pwmValueA);
	analogWrite(input2Pin, pwmValueB);
	analogWrite(input3Pin, pwmValueC);
}

// call me as often as possible
void BLDCController::loop() {

	// turn reference angle along the set speed
	float timePassed_s = turnReferenceAngle();

	// read the current encoder value
	readEncoder();

	// compute the angle of the magnetic field
	// - starting point is reference angle
	// - compute difference as given by the encoder
	// - feed into PI controller, result is magnetic field
	// set torque linear to error
	float errorAngle = referenceAngle - encoderAngle;

	// carry out PI controller to compute advance angle
	advanceAngleError = pid_k*errorAngle;
	advanceAnglePhase += errorAngle * timePassed_s;
	advanceAnglePhase = constrain(advanceAnglePhase, -radians(30), radians(30)); // limit error integral by 30°
	float i_out = pid_i * advanceAnglePhase;
	float advanceAngle = advanceAngleError + i_out;
	advanceAngle = constrain(advanceAngle, -radians(30), radians(30)); 			// limit outcome by 30°

	// outcome of pid controller is advance angle
	magneticFieldAngle = referenceAngle + advanceAngle;

	// if the motor is not able to follow the magnetic field , limit the reference angle accordingly
	referenceAngle = constrain(referenceAngle, encoderAngle - radians(30), encoderAngle  + radians(30));

	// recompute speed, since set speed might not be achieved
	currentSpeed = (referenceAngle-lastReferenceAngle)/timePassed_s/radians(360);
	lastReferenceAngle = referenceAngle; // required to compute speed

	// send new pwm value to motor
	// set torque proportional to advanceAngleError
	sendPWMDuty();

	/*
	Serial1.print("time=");
	Serial1.print(timePassed_s);
	Serial1.print(" v=");
	Serial1.print(currentSpeed);
	Serial1.print(" r=");
	Serial1.print(degrees(referenceAngle));
	Serial1.print("° e=");
	Serial1.print(degrees(encoderAngle));
	Serial1.print("° err=");
	Serial1.print(errorAngle);
	Serial1.print(" int=");
	Serial1.print(advanceAnglePhase);
	Serial1.print(" iout=");
	Serial1.print(degrees(i_out));
	Serial1.print(" aae=");
	Serial1.print(degrees(advanceAngleError));
	Serial1.print(" a=");
	Serial1.print(degrees(advanceAngle));
	Serial1.print("° m=");
	Serial1.print(degrees(magneticFieldAngle));
	Serial1.print("° ut=");
	Serial1.print(targetTorque);
	Serial1.print(" torque=");
	Serial1.print(torque);
	Serial1.println();
	*/

}

void BLDCController::setTorque(float newTorque) {
	targetTorque = newTorque;
}

void BLDCController::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetSpeed = speed;
	targetAcc = acc;
}

float BLDCController::getSpeed() {
	return currentSpeed;
}

float BLDCController::getRevolution() {
	return referenceAngle;
}


void BLDCController::enable(bool doit) {
	isEnabled = doit;
	if (isEnabled) {
		// reset speed before enabling. Dont reset the target speed
		currentSpeed = 0;

		// startup procedure to find the angle of the motor's rotor
		// - turn magnetic field with min torque (120° max) until encoder recognizes a significant movement
		// - turn in other direction until this movement until encoder gives original position
		// if the encoder does not indicate a movement, increase torque and try again

		encoderAngle = 0;
		magneticFieldAngle = 0;
		referenceAngle = 0;
		advanceAnglePhase = 0;
		advanceAngleError = 0;
		currentSpeed = 0;
		targetTorque = 0;

		// enable driver, but PWM has no duty cycle yet.
		sendPWMDuty();
		digitalWrite(enablePin, HIGH);
		delay(50); // settle

		// startup calibration works by turning the magnetic field until the encoder
		// indicates the rotor being aligned with the field
		// During calibration, run a loop that
		// - measure the encoder
		// - turn the magnetic field in the direction of the deviation as indicated by the encoder
		// - if the encoders indicates no movement, increase torque
		// quit the loop if torque is above a certain threshold with encoder at 0
		// end calibration by setting the current reference angle to the measured rotors position
		float lastLoopEncoderAngle = 0;
		targetTorque = 0;
		while ((targetTorque < 0.2)) { // quit when above 20% torque
			// P controller with p=4.0
			magneticFieldAngle -= encoderAngle*4.0; // this is a P-controller that turns the magnetic field towards the direction of the encoder
			sendPWMDuty();
			delay(20);
			readEncoder();
			float encoderLoopDiff = encoderAngle - lastLoopEncoderAngle;
			// if encoder indicates no movement, we can increase torque a bit.
			if (abs(encoderLoopDiff) < radians(0.5))
				targetTorque += 0.005;
			lastLoopEncoderAngle = encoderAngle;

		}

		referenceAngle = magneticFieldAngle;
		encoderAngle = magneticFieldAngle;

		// set default torque
		targetTorque = minTorque;
		sendPWMDuty();
	}
	else
		digitalWrite(enablePin, LOW);
}

void BLDCController::printHelp() {
	Serial1.println("BLDC controller");
	Serial1.println("0 - stop");
	Serial1.println("+ - inc speed");
	Serial1.println("- - dec speed");
	Serial1.println("* - inc acc");
	Serial1.println("_ - dec acc");
	Serial1.println("r - revert direction");
	Serial1.println("T/t - increase torque");
	Serial1.println("P/p - increase PIs controller p");
	Serial1.println("I/i - increase PIs controller i");

	Serial1.println("e - enable");

	Serial1.println("ESC");
}

void BLDCController::runMenu() {
	printHelp();
	TimePassedBy timer(1);

	while (true) { // terminates with return
		if (timer.isDue())
			loop();
		char ch = Serial1.read();
		bool cmd = true;
		switch (ch) {
		case '0':
			menuSpeed = 0;
			setSpeed(menuSpeed,  menuAcc);
			break;
		case '+':
			if (abs(menuSpeed) < 2)
				menuSpeed += 0.05;
			else
				menuSpeed += 1.0;

			setSpeed(menuSpeed,  menuAcc);
			break;
		case '-':
			if (abs(menuSpeed) < 2)
				menuSpeed -= 0.05;
			else
				menuSpeed -= 1.0;
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
		case 'P':
			pid_k += 0.02;
			Serial1.print("p=");
			Serial1.println(pid_k);

			break;
		case 'p':
			pid_k -= 0.02;
			Serial1.print("p=");
			Serial1.println(pid_k);

			break;

		case 'I':
			pid_i += 0.02;
			Serial1.print("i=");
			Serial1.println(pid_i);

			break;
		case 'i':
			pid_i -= 0.02;
			Serial1.print("i=");
			Serial1.println(pid_i);

			break;

		case 'e':
			menuEnable = menuEnable?false:true;
			enable(menuEnable);
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
			Serial1.print(" actual v=");
			Serial1.print(getSpeed());
			Serial1.print(" a=");
			Serial1.print(menuAcc);
			Serial1.print(" T=");
			Serial1.print(menuTorque);
			Serial1.print(" t=");
			Serial1.print(micros());
			if (menuEnable)
				Serial1.print(" enabled");
			else
				Serial1.print(" disabled");

			Serial1.println(" >");
		}
	}
}

