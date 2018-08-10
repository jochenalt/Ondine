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


float BLDCController::pid_k = 0.8;
float BLDCController::pid_i = 0.05;

const float maxSpeed = 50.0; // [rev/s]
const float minTorque = 0.1; // [PWM ratio]

// max PWM value is (1<<pwmResolution)-1
const int pwmResolution = 8;

// array to store pre-computed values of space vector wave form (SVPWM)
const int svpwmArraySize = 244;
int svpwmTable[svpwmArraySize];

void precomputeSVPMWave() {
	const int maxPWMValue = (1<<pwmResolution)-1;
	const float spaceVectorFactor = 1.13; // empiric to reach full pwm scale
	static boolean initialized = false;
	if (!initialized) {
		for (int i = 0;i<svpwmArraySize;i++) {
			float angle = float(i)/ float(svpwmArraySize) * (M_PI*2.0);
			float phaseA = sin(angle);
			float phaseB = sin(angle + M_PI*2.0/3.0);
			float phaseC = sin(angle + M_PI*4.0/3.0);
			float voff = (min(phaseA, min(phaseB, phaseC)) + max(phaseA, max(phaseB, phaseC)))/2.0;
			float pwmSpaceVectorValue =  ((phaseA - voff)/2.0*spaceVectorFactor + 0.5)*maxPWMValue;
			float pwmSinValue =  (phaseA/2.0 + 0.5)*maxPWMValue;

			/*
			Serial1.print("i=");
			Serial1.print(i);
			Serial1.print(" voff=");
			Serial1.print(voff);

			Serial1.print(" SPPWM=");
			Serial1.print(pwmSpaceVectorValue);
			Serial1.print(" SPWM=");
			Serial1.println(pwmSinValue);
*/
			if (pwmSpaceVectorValue<10)
				pwmSpaceVectorValue = 0;
			svpwmTable[i] =  pwmSpaceVectorValue;
			svpwmTable[i] =  pwmSinValue;

		}
		initialized = true;
	}
}

int BLDCController::getPWMValue(float angle_rad) {

	// torque is increased with speed. When reaching max speed, torque becomes 1
	// torque increases along the pid's outcome
	torque = targetTorque + (1.0-targetTorque)*( min(abs(currentSpeed)/maxSpeed + abs(advanceAngle)/(M_PI/12.0),1.0) );

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
}

BLDCController::~BLDCController() {
}


void BLDCController::setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {

	// initialize precomputed spvm values (only once)
	precomputeSVPMWave();

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
	digitalWrite(enablePin, HIGH);

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

	// check for overflow on micros() (happens every 70 minutes)
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
		long encoderPosition= encoder->read();
		encoderAngle += (lastEncoderPosition - encoderPosition)/encoderCPR;
		lastEncoderPosition = encoderPosition;
	}
}

// set the pwm values matching the current magnetic field angle
void BLDCController::setPWM() {
	int pwmValueA, pwmValueB, pwmValueC;
	getPWMValues (pwmValueA, pwmValueB, pwmValueC);
	analogWrite(input1Pin, pwmValueA);
	analogWrite(input2Pin, pwmValueB);
	analogWrite(input3Pin, pwmValueC);
}

// call me as often as possible
void BLDCController::loop() {
	float timePassed_s = turnReferenceAngle();
	readEncoder();


	// compute the angle of the magnetic field
	// - starting point is reference angle
	// - compute difference as given by the encoder
	// - feed into PI controller, result is magnetic field
	// set torque linear to error
	float errorAngle = encoderAngle - referenceAngle;

	// carry out PI controller to compute advance angle
	float p_out = pid_k*errorAngle;
	errorAngleIntegral += errorAngle * timePassed_s;
	errorAngleIntegral = constrain(errorAngleIntegral, - M_PI/12.0, M_PI/12.0); // limit error integral by 30°
	float i_out = pid_i *errorAngleIntegral;
	advanceAngle = p_out + i_out;
	advanceAngle = constrain(advanceAngle, - M_PI/12.0, M_PI/12.0); 				// limit outcome by 30°

	// outcome of pid controller is advance angle
	magneticFieldAngle = referenceAngle + advanceAngle;




	Serial1.print("t=");
	Serial1.print(timePassed_s);
	Serial1.print(" v=");
	Serial1.print(currentSpeed);
	Serial1.print(" r=");
	Serial1.print(referenceAngle);
	Serial1.print(" e=");
	Serial1.print(encoderAngle);
	Serial1.print(" m=");
	Serial1.print(magneticFieldAngle);
	Serial1.print(" ut=");
	Serial1.print(targetTorque);
	Serial1.print(" t=");
	Serial1.print(torque);

	Serial1.println();


	setPWM();
}

void BLDCController::setTorque(float newTorque) {
	targetTorque = newTorque;
}

void BLDCController::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetSpeed = speed;
	targetAcc = acc;
}


void BLDCController::enable(bool doit) {
	isEnabled = doit;
	if (isEnabled) {
		// reset speed before enabling. Dont reset the target speed
		currentSpeed = 0;

		digitalWrite(enablePin, HIGH);

		// startup procedure to find the angle of the motor's rotor
		// - turn magnetic field with min torque (120° max) until encoder recognizes a significant movement
		// - turn in other direction until this movement until encoder gives original position
		// if the encoder does not indicate a movement, increase torque and try again

		encoderAngle = 0;
		magneticFieldAngle = 0;
		referenceAngle = 0;
		targetTorque = minTorque;
		/*
		const float significantAngle = 5.0/360.0*2.0*M_PI;

		Serial.print("calibration ");
		readEncoder();
		targetTorque = minTorque;
		bool encoderMoved = false;
		while (!encoderMoved) {
			while ((abs(encoderAngle) < significantAngle) && (encoderAngle < 2.0*M_PI/3.0) && (magneticFieldAngle < 2.0*M_PI/3)) {
				magneticFieldAngle += significantAngle;
				setPWM();
				delay(1000/ (360.0/significantAngle)); // wait such that we ave 1 revolution per second
				readEncoder();
			}
			targetTorque += 0.05;
		}
		if (encoderAngle > significantAngle) {
			Serial1.println("done");

			// let the regular loop turn back to original position
			referenceAngle = encoderAngle - significantAngle;
		} else {
			Serial1.println("impossible");
		}
		*/

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
			break;
		case 'p':
			pid_k -= 0.02;
			break;

		case 'I':
			pid_i += 0.02;
			break;
		case 'i':
			pid_i -= 0.02;
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

