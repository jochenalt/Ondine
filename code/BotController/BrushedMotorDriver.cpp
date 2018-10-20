/*
 * BrushedMotorDriver.cpp
 *
 *  Created on: 08.09.2018
 *      Author: JochenAlt
 */

#include <BrushedMotorDriver.h>
#include <MenuController.h>

#include <setup.h>
#include <BotMemory.h>

void BrushedMotorDriver::setup(MenuController* menuCtrl) {
	registerMenuController(menuCtrl);
}

void BrushedMotorDriver::setupMotor(int enablePin,int in1Pin, int in2Pin, int currentSensePin) {
	this->enablePin = enablePin;
	this->in1Pin = in1Pin;
	this->in2Pin = in2Pin;
	this->currentSensePin = currentSensePin;

	// has to be pwm pins
	pinMode(enablePin, OUTPUT);
	pinMode(in1Pin, OUTPUT);
	pinMode(in2Pin, OUTPUT);
	pinMode(currentSensePin, INPUT);

	// setup L6234 input PWM pins
	analogWriteResolution(pwmResolution);

	digitalWrite(enablePin, LOW); // start with disabled motor
	analogWriteFrequency(in1Pin, 20000);

	lastLoopCall_ms = 0;
	referenceSpeed = 0;
	referenceAngle = 0;
	currentMotorPower = 0;
	enabled = false;
}

void BrushedMotorDriver::setupEncoder(int EncoderAPin, int EncoderBPin, int CPR) {
	this->encoderAPin = EncoderAPin;
	this->encoderBPin = EncoderBPin;
	this->CPR = CPR;

	encoder = new Encoder(EncoderAPin, EncoderBPin);
}

void BrushedMotorDriver::readEncoder() {
	if (encoder == NULL) {
		// without encoder assume perfect motor
		encoderAngle = referenceAngle;
	}
	else {
		// find encoder position and increment the encoderAngle accordingly
		int32_t encoderPosition= encoder->read();
		encoderAngle += ((float)(lastEncoderPosition - encoderPosition))/(float)CPR*TWO_PI;
		lastEncoderPosition = encoderPosition;
	}
}

float BrushedMotorDriver::getMotorAngle() {
	return encoderAngle;
}

void BrushedMotorDriver::readCurrentSense() {
	currentCurrent =  (float)analogRead(currentSensePin)/1024.0 / 0.525;
}

float BrushedMotorDriver::getCurrentCurrent() {
	return currentCurrent;
}

void BrushedMotorDriver::loop() {
	// turn reference angle
	uint32_t now = millis();
	if (lastLoopCall_ms > 0) {
		// change anything with max frequency of 100 Hz
		if ((enabled) && (now >= lastLoopCall_ms + 1000/SampleFrequency)) {
			float dT = (1.0/1000.0)*(now - lastLoopCall_ms);
			lastLoopCall_ms = now;

			// compute reference angle in [rad]
			referenceAngle += dT * referenceSpeed * TWO_PI;

			// what is the real angle delivered by optical encoder
			readEncoder();

			// compare motor angle with measured encoder angle
			float angleError = encoderAngle - referenceAngle; // [rad]

			// PID controller delivers power ratio to be sent to motor
			// since the encoder is quite coarse with 48 CPR, controller must be relatively low.
			float outputAngle = pid.update(memory.persistentMem.motorControllerConfig.pid_lifter, angleError, dT, -radians(30), radians(30));
			currentMotorPower = constrain(outputAngle/radians(30), -1.0, +1.0);
			setMotorPower(currentMotorPower);
			readCurrentSense();

			if (logValues) {
				logger->print("dT=");
				logger->print(dT);
				logger->print(" ref=");
				logger->print(degrees(referenceAngle));
				logger->print(" enc=");
				logger->print(degrees(encoderAngle));

				logger->print(" err=");
				logger->print(degrees(angleError));
				logger->print(" pow=");
				logger->print(currentMotorPower);

				logger->print(" int=");
				logger->print(pid.integrativeError);

				logger->print(" t=");
				logger->print(currentCurrent*1000);
				logger->print("mA");
				logger->println();
			}
		}
	} else
		lastLoopCall_ms = now;
}


void BrushedMotorDriver::setMotorSpeed(float speed) {
	referenceSpeed = speed;
}

void BrushedMotorDriver::setMotorPower(float powerRatio) {
	float torque = constrain(powerRatio, -1.0, 1.0);
	bool direction = (torque > 0);
	// analogWrite(in2Pin, speed/MaxSpeed*((1<<pwmResolution)-1) );
	int maxPWM = ((1<<pwmResolution)-1);
	int pwmValue = abs(torque*maxPWM);
	if (!direction)
		pwmValue = maxPWM - pwmValue;

	digitalWrite(in2Pin, (direction)?LOW:HIGH);
	analogWrite(in1Pin, pwmValue);
}

float BrushedMotorDriver::getMotorSpeed() {
	return 0;
}

void BrushedMotorDriver::enable(bool doIt) {
	enabled = doIt;
	digitalWrite(enablePin, doIt?HIGH:LOW);
	if (enabled) {
		referenceAngle = encoderAngle;
		currentMotorPower = 0;
		setMotorPower(currentMotorPower);
		lastLoopCall_ms = 0;
	}
}

void BrushedMotorDriver::printHelp() {
	command->println();

	command->println("brushed motor menu");
	command->println();
	command->println("0   - stop");
	command->println("P/p - controller's P factor");
	command->println("I/i - controller's I factor");

	command->println("+   - inc speed");
	command->println("-   - dec speed");
	command->println("r   - revert direction");
	command->println("l   - log values");
	command->println("e   - enable");

	command->println("ESC");
}

void BrushedMotorDriver::menuLoop(char ch) {

		bool cmd = true;
		switch (ch) {
		case '0':
			menuSpeed = 0;
			setMotorSpeed(menuSpeed);
			break;
		case 'P':
			memory.persistentMem.motorControllerConfig.pid_lifter.Kp += 0.01;
			break;
		case 'p':
			memory.persistentMem.motorControllerConfig.pid_lifter.Kp -= 0.01;
			break;
		case 'I':
			memory.persistentMem.motorControllerConfig.pid_lifter.Ki += 0.00001;
			break;
		case 'i':
			memory.persistentMem.motorControllerConfig.pid_lifter.Ki -= 0.00001;
			break;

		case '+':
			if (abs(menuSpeed) < 2)
				menuSpeed += 0.05;
			else
				menuSpeed += 1.0;

			setMotorSpeed(menuSpeed);
			break;
		case '-':
			if (abs(menuSpeed) < 2)
				menuSpeed -= 0.05;
			else
				menuSpeed -= 1.0;
			setMotorSpeed(menuSpeed);
			break;
		case 'l':
			logValues = !logValues;
			break;
		case 'e':
			menuEnable = menuEnable?false:true;
			enable(menuEnable);
			break;
		case 'h':
			printHelp();
			break;
		default:
			cmd = false;
			break;
		}
		if (cmd) {
			command->print("v=");
			command->print(menuSpeed);
			command->print(" actual v=");
			command->print(getMotorSpeed());
			command->print(" actual angle=");
			command->print(getMotorAngle());
			command->print(" PID=(");
			command->print(memory.persistentMem.motorControllerConfig.pid_lifter.Kp,5);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_lifter.Ki,5);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_lifter.Kd,5);
			command->print(")");


			command->print(micros());
			if (menuEnable)
				command->print(" enabled");
			else
				command->print(" disabled");
			command->println();

			command->print(">");
		}
}

