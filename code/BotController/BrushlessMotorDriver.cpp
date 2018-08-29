/*
 * BLDCController.cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <Util.h>
#include <BotMemory.h>
#include <BrushlessMotorDriver.h>

#include <ClassInterrupt.h>
#include <utilities/TimePassedBy.h>


const float maxAdvanceAngle = radians(30); 			// [rad], has been found out empirically
const float maxSpeed = 60; 							// [rev/s]

// max PWM value is (1<<pwmResolution)-1
const int pwmResolution = 10;

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

			svpwmTable[i] =  pwmSpaceVectorValue;
			svpwmTable[i] =  pwmSinValue;

		}
		initialized = true;
	}
}

int BrushlessMotorDriver::getPWMValue(float torque, float angle_rad) {
	// map input angle to 0..2*PI
	if (angle_rad < 0)
		angle_rad += (int)(abs(angle_rad)/(2.0*M_PI) + 1.0)*2.0*M_PI;

	int angleIndex = ((int)(angle_rad / ( 2.0*M_PI) * svpwmArraySize)) % svpwmArraySize;
	if ((angleIndex <0) || (angleIndex > svpwmArraySize))
		fatalError("cvpwm table look up error");

	return  torque * svpwmTable[angleIndex];
}

BrushlessMotorDriver::BrushlessMotorDriver() {
	// initialize precomputed spvm values (only once)
	precomputeSVPMWave();
}

void estimateControllerGain(float K_s /* amplification */, float T_u /* delay */, float T_g /* control time */, float &K_p, float &K_i, float &K_d)  {
	// Ziegler and Nicols, according to https://rn-wissen.de/wiki/index.php/Regelungstechnik
	K_p = (0.9 / K_s) * (T_g / T_u);
	K_i = K_p / (3.3 * T_u);
	K_d = K_p*0.5*T_u;
}

void BrushlessMotorDriver::setup(MenuController* menuCtrl) {
	registerMenuController(menuCtrl);
}

void BrushlessMotorDriver::setupMotor( int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {
	// there's only one enable pin that has a short cut to EN1, EN2, and EN3 from L6234
	enablePin = EnablePin;

	input1Pin = Input1Pin;
	input2Pin = Input2Pin;
	input3Pin = Input3Pin;

	// setup L6234 input PWM pins
	analogWriteResolution(pwmResolution);

	// choose the frequency that it just can't be heard
	analogWriteFrequency(input1Pin, 50000);
	analogWriteFrequency(input2Pin, 50000);
	analogWriteFrequency(input3Pin, 50000);

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

void BrushlessMotorDriver::setupEncoder(int EncoderAPin, int EncoderBPin, int CPR) {
	encoderAPin = EncoderAPin;
	encoderBPin = EncoderBPin;

	encoderCPR = CPR;
	encoder = new Encoder(encoderAPin, encoderBPin);
}

// turn magnetic field of the motor according to current speed
void BrushlessMotorDriver::setMagneticFieldAngle(float angle) {
	// increase angle of magnetic field
	magneticFieldAngle = angle;
}

float BrushlessMotorDriver::turnReferenceAngle() {
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
	referenceAngle += currentSpeed * timePassed_s * TWO_PI;

	return timePassed_s;
}

void BrushlessMotorDriver::readEncoder() {
	if (encoder == NULL) {
		// without encoder assume perfect motor
		encoderAngle = referenceAngle;
	}
	else {
		// find encoder position and increment the encoderAngle accordingly
		int32_t encoderPosition= encoder->read();
		encoderAngle += ((float)(lastEncoderPosition - encoderPosition))/(float)encoderCPR*TWO_PI/4.0;
		lastEncoderPosition = encoderPosition;
	}
}

// set the pwm values matching the current magnetic field angle
void BrushlessMotorDriver::sendPWMDuty(float torque) {
	int pwmValueA = getPWMValue(torque, magneticFieldAngle);
	int pwmValueB = getPWMValue(torque, magneticFieldAngle + 1.0*TWO_PI/3.0);
	int pwmValueC = getPWMValue(torque, magneticFieldAngle + 2.0*TWO_PI/3.0);
	analogWrite(input1Pin, pwmValueA);
	analogWrite(input2Pin, pwmValueB);
	analogWrite(input3Pin, pwmValueC);
}


// call me as often as possible
void BrushlessMotorDriver::loop() {
	// run only if at least one ms passed
	uint32_t now = millis();
	if (now - lastCall < 1)
		return;
	lastCall = now;

	// turn reference angle along the set speed
	float timePassed_s = turnReferenceAngle();
	// logger->print("loop t=");
	// logger->print(timePassed_s);
	// logger->print(" r=");
	// logger->print(referenceAngle);
	// logger->print("  ");

	// read the current encoder value
	readEncoder();

	// set the advance angle linear to speed (according to "Advance Angle Calculation for
	// Improvement of the Torque-to Current Ratio of Brushless DC Motor Drives")
	advanceAngle = currentSpeed/maxSpeed*maxAdvanceAngle;

	// compute position error
	float errorAngle = referenceAngle - encoderAngle;

	// in case of deceleration, use much smaller delay time, which leads to higher PI controller factors
	// according to "Brushless Control Made Easy.pdf"
	float controllerDecelerationFactor = 1.0; // factor of acceleration delay time compared to deceleration
	if (((errorAngle < 0 ) && (actualSpeed > 0)) ||
		((errorAngle > 0 ) && (actualSpeed < 0)))
		controllerDecelerationFactor = MaxDecelerationFactor;

	// carry out PI controller with integrative gain increasing with speed
	float controlOutput = pid.update(memory.persistentMem.motorControllerConfig.pid, errorAngle, timePassed_s, -radians(maxAdvanceAngle),radians(maxAdvanceAngle));

	// magnetic force is proportional to distance^2, i.e. torque is proportional to sqr(tan(advanceAngle))
	// control output plays with that torque
	// assume tan(x) = x
	float torque = sqr(abs(advanceAngle)/maxAdvanceAngle)*(controlOutput/maxAdvanceAngle);

	// if controller shows a small error, stay at ideal advance angle.
	// the higher the error, the more the advance angle is reduce in order to get more physical torque (at the same time, the torque ratio is increased)
	// even turn sign of advance angle in case of negative error (i.e. the motor needs to break)
	// FIXME: If the control output is suddenly negativ, then the advance angle gets fully negative. At high speed, this
	// might give some jerks
	advanceAngle = sgn(controlOutput)*advanceAngle*(1.0-abs(controlOutput)/maxAdvanceAngle);

	// outcome of pid controller is advance angle
	magneticFieldAngle = encoderAngle + advanceAngle;

	// if the motor is not able to follow the magnetic field , limit the reference angle accordingly
	referenceAngle = constrain(referenceAngle, encoderAngle - maxAdvanceAngle, encoderAngle  + maxAdvanceAngle);
	// recompute speed, since set speed might not be achieved

	actualSpeed = (referenceAngle-lastReferenceAngle)/TWO_PI/timePassed_s;
	lastReferenceAngle = referenceAngle; // required to compute speed

	// send new pwm value to motor
	sendPWMDuty(min(abs(torque),1.0));

	static uint32_t lastoutput = 0;

	if (millis() - lastoutput >  100) {
		lastoutput = millis();
	command->print("time=");
	command->print(timePassed_s*1000.0);
	command->print("ms v=");
	command->print(currentSpeed);
	command->print("/");
	command->print(actualSpeed);
	command->print(" r=");
	command->print(degrees(referenceAngle));
	command->print("° e=");
	command->print(degrees(encoderAngle));
	command->print("° err=");
	command->print(degrees(errorAngle));
	command->print(" control=");
	command->print(degrees(controlOutput));
	command->print(" aa=");
	command->print(degrees(advanceAngle));
	command->print("° tq=");
	command->print(torque);
	command->print("m=");
	command->print(degrees(magneticFieldAngle));
	command->println();
	}

}

void BrushlessMotorDriver::setMotorSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetSpeed = speed;
	targetAcc = acc;
}

float BrushlessMotorDriver::getMotorSpeed() {
	return actualSpeed;
}

float BrushlessMotorDriver::getIntegratedMotorAngle() {
	return referenceAngle;
}


void BrushlessMotorDriver::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	setMotorSpeed(speed/GearBoxRatio,acc);
}

float BrushlessMotorDriver::getSpeed() {
	return actualSpeed*GearBoxRatio;
}

float BrushlessMotorDriver::getIntegratedAngle() {
	return referenceAngle*GearBoxRatio;
}

void BrushlessMotorDriver::enable(bool doit) {
	isEnabled = doit;
	if (isEnabled) {

		// startup procedure to find the angle of the motor's rotor
		// - turn magnetic field with min torque (120° max) until encoder recognizes a significant movement
		// - turn in other direction until this movement until encoder gives original position
		// if the encoder does not indicate a movement, increase torque and try again

		encoderAngle = 0;
		magneticFieldAngle = 0;
		referenceAngle = 0;
		currentSpeed = 0;
		actualSpeed = 0;
		advanceAngle = 0;
		lastEncoderPosition = 0;

		// enable driver, but PWM has no duty cycle yet.
		sendPWMDuty(0);
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
		float targetTorque = 0;
		while ((targetTorque < 0.2)) { // quit when above 20% torque
			// P controller with p=4.0
			magneticFieldAngle -= encoderAngle*1.0; // this is a P-controller that turns the magnetic field towards the direction of the encoder

			sendPWMDuty(targetTorque);
			delay(20);
			readEncoder();
			float encoderLoopDiff = encoderAngle - lastLoopEncoderAngle;
			// if encoder indicates no movement, we can increase torque a bit until the motor moves
			if (abs(encoderLoopDiff) < radians(1.0))
				targetTorque += 0.005;
			lastLoopEncoderAngle = encoderAngle;
		}

		referenceAngle = magneticFieldAngle;
		encoderAngle = magneticFieldAngle;
		lastReferenceAngle = magneticFieldAngle;
		sendPWMDuty(0);
	}
	else
		digitalWrite(enablePin, LOW);
}


void BrushlessMotorDriver::printHelp() {
	command->println();

	command->println("brushless motor menu");
	command->println();
	command->println("0 - stop");
	command->println("+ - inc speed");
	command->println("- - dec speed");
	command->println("* - inc acc");
	command->println("_ - dec acc");
	command->println("r - revert direction");
	command->println("T/t - increase torque");
	command->println("P/p - increase PIs controller p");
	command->println("I/i - increase PIs controller i");

	command->println("e - enable");

	command->println("ESC");
}

void BrushlessMotorDriver::menuLoop(char ch) {

		bool cmd = true;
		switch (ch) {
		case '0':
			menuSpeed = 0;
			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case '+':
			if (abs(menuSpeed) < 2)
				menuSpeed += 0.05;
			else
				menuSpeed += 1.0;

			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case '-':
			if (abs(menuSpeed) < 2)
				menuSpeed -= 0.05;
			else
				menuSpeed -= 1.0;
			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case '*':
			menuAcc++;
			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case '_':
			menuAcc--;
			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case 'r':
			menuSpeed = -menuSpeed;
			setMotorSpeed(menuSpeed,  menuAcc);
			break;
		case 'P':
			memory.persistentMem.motorControllerConfig.pid.Kp  += 0.02;
			command->print("Kp=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kp);

			break;
		case 'p':
			memory.persistentMem.motorControllerConfig.pid.Kp -= 0.02;
			command->print("Kp=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kp);
			break;
		case 'D':
			memory.persistentMem.motorControllerConfig.pid.Kd  += 0.02;
			command->print("Kd=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kd);

			break;
		case 'd':
			memory.persistentMem.motorControllerConfig.pid.Kd -= 0.02;
			command->print("Kd=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kd);
			break;
		case 'I':
			memory.persistentMem.motorControllerConfig.pid.Ki  += 0.02;
			command->print("Ki=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Ki);

			break;
		case 'i':
			memory.persistentMem.motorControllerConfig.pid.Ki -= 0.02;
			command->print("Ki=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Ki);
			break;
		case 'Z':
			memory.persistentMem.motorControllerConfig.pid.K_s  += 0.02;
			command->print("amplification=");
			command->println(memory.persistentMem.motorControllerConfig.pid.K_s);
			command->print("deadtime=");
			command->println(memory.persistentMem.motorControllerConfig.pid.T_u);
			command->print("controltime=");
			command->println(memory.persistentMem.motorControllerConfig.pid.T_g);
			memory.persistentMem.motorControllerConfig.pid.zieglerAndNicols();
			command->print("PID=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kp);
			command->print(",");
			command->println(memory.persistentMem.motorControllerConfig.pid.Ki);
			command->print(",");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kd);
			command->println(")");

			break;
		case 'z':
			memory.persistentMem.motorControllerConfig.pid.K_s  -= 0.02;
			command->print("amplification=");
			command->println(memory.persistentMem.motorControllerConfig.pid.K_s);
			command->print("deadtime=");
			command->println(memory.persistentMem.motorControllerConfig.pid.T_u);
			command->print("controltime=");
			command->println(memory.persistentMem.motorControllerConfig.pid.T_g);
			memory.persistentMem.motorControllerConfig.pid.zieglerAndNicols();
			command->print("PID=");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kp);
			command->print(",");
			command->println(memory.persistentMem.motorControllerConfig.pid.Ki);
			command->print(",");
			command->println(memory.persistentMem.motorControllerConfig.pid.Kd);
			command->println(")");

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
			command->print(" a=");
			command->print(menuAcc);
			command->print(" T=");
			command->print(menuTorque);
			command->print(" t=");
			command->print(" ref=");
			command->print(degrees(referenceAngle));

			command->print(micros());
			if (menuEnable)
				command->print(" enabled");
			else
				command->print(" disabled");
			command->println();

			command->print(">");
		}
}

