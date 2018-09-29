/*
 * BrushedMotorDriver.cpp
 *
 *  Created on: 08.09.2018
 *      Author: JochenAlt
 */

#include <BrushedMotorDriver.h>
#include <MenuController.h>

#include <setup.h>

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
		encoderAngle += ((float)(lastEncoderPosition - encoderPosition))/(float)CPR*TWO_PI/4.0;
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
	readEncoder();
	readCurrentSense();
}

void BrushedMotorDriver::setMotorSpeed(float speed) {
	float ratio = abs(speed/10);
	bool direction = (speed > 0);
	// analogWrite(in2Pin, speed/MaxSpeed*((1<<pwmResolution)-1) );
	int maxPWM = ((1<<pwmResolution)-1);
	int pwmValue = abs(ratio*maxPWM);
	if (!direction)
		pwmValue = maxPWM - pwmValue;

	logger->print("ratio");
	logger->println(ratio);
	logger->print("pwm");
	logger->println(pwmValue);
	logger->print("direction");
	logger->println(direction);

	digitalWrite(in2Pin, (direction)?LOW:HIGH);

	analogWrite(in1Pin, pwmValue);

}

float BrushedMotorDriver::getMotorSpeed() {
	return 0;
}

void BrushedMotorDriver::enable(bool doIt) {
	digitalWrite(enablePin, doIt?HIGH:LOW);
}

void BrushedMotorDriver::printHelp() {
	command->println();

	command->println("brushed motor menu");
	command->println();
	command->println("0 - stop");
	command->println("+ - inc speed");
	command->println("- - dec speed");
	command->println("r - revert direction");
	command->println("e - enable");

	command->println("ESC");
}

void BrushedMotorDriver::menuLoop(char ch) {

		bool cmd = true;
		switch (ch) {
		case '0':
			menuSpeed = 0;
			setMotorSpeed(menuSpeed);
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

			command->print(micros());
			if (menuEnable)
				command->print(" enabled");
			else
				command->print(" disabled");
			command->println();

			command->print(">");
		}
}

