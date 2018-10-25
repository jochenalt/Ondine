/*
 * BrushlessMotorDriver.cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <setup.h>
#include <Util.h>
#include <BotMemory.h>
#include <BrushlessMotorDriver.h>

#include <TimePassedBy.h>

const float maxAngleError = radians(30);
const float maxAdvancePhaseAngle = radians(10);
const float RevPerSecondPerVolt = 5;							// motor constant of Maxon EC max 40 W
const float voltage = 12;										// [V] coming from the battery to server the motors
const float maxRevolutionSpeed = voltage*RevPerSecondPerVolt; 	// [rev/s]


float sigmoid(float gain, float x) {
	return 2.0/(1.0 + exp(gain * x)) - 1.0;
}

// array to store pre-computed values of space vector wave form (SVPWM)
// array size is choosen by having a maximum difference of 1% in two subsequent table items,
// i.e. we have a true running with a precision of 1%. The rest is compensated by the optical
// encoder with a precision of 0.1%
#define svpwmArraySize 244
int svpwmTable[svpwmArraySize];

void precomputeSVPMWave() {
	const int maxPWMValue = (1<<pwmResolution)-1;
	const float spaceVectorScaleUpFactor = 1.15; // empiric value to reach full pwm scale
	static boolean initialized = false;
	if (!initialized) {
		for (int i = 0;i<svpwmArraySize;i++) {
			float angle = float(i)/ float(svpwmArraySize) * (TWO_PI);
			float phaseA = sin(angle);
			float phaseB = sin(angle + M_PI*2.0/3.0);
			float phaseC = sin(angle + M_PI*4.0/3.0);

			// trick to avoid the switch of 6 phases everyone else is doing, neat, huh?
			float voff = (min(phaseA, min(phaseB, phaseC)) + max(phaseA, max(phaseB, phaseC)))/2.0;

			float pwmSpaceVectorValue =  (phaseA - voff)/2.0*spaceVectorScaleUpFactor*maxPWMValue;
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
		angle_rad += (int)(abs(angle_rad)/TWO_PI + 1.0)*TWO_PI;

	int angleIndex = ((int)(angle_rad / TWO_PI * svpwmArraySize)) % svpwmArraySize;
	if ((angleIndex < 0) || (angleIndex > svpwmArraySize))
		fatalError("getPWMValue: idx out of bounds");

	return  torque * svpwmTable[angleIndex];
}

BrushlessMotorDriver::BrushlessMotorDriver() {
	// initialize precomputed spvm values
	// first invocation does the initialization
	precomputeSVPMWave();
}

void BrushlessMotorDriver::setup( int motorNo, MenuController* menuCtrl) {
	this->motorNo = motorNo;
	registerMenuController(menuCtrl);
}

void BrushlessMotorDriver::setupMotor(int EnablePin, int Input1Pin, int Input2Pin, int Input3Pin) {
	// there's only one enable pin that has a short cut to EN1, EN2, and EN3 from L6234
	enablePin = EnablePin;

	input1Pin = Input1Pin;
	input2Pin = Input2Pin;
	input3Pin = Input3Pin;

	// setup L6234 input PWM pins
	analogWriteResolution(pwmResolution);

	// choose a frequency that just can't be heard
	analogWriteFrequency(input1Pin, 50000);
	analogWriteFrequency(input2Pin, 50000);
	analogWriteFrequency(input3Pin, 50000);

	// these pins have to have PWM functionality
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);
	pinMode(input1Pin, OUTPUT);

	// enable all enable lines at once (Drotek L6234 board has all enable lines connected)
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, LOW); // start with disabled motor, ::enable turns it on
}

void BrushlessMotorDriver::setupEncoder(int EncoderAPin, int EncoderBPin, int CPR) {
	encoderAPin = EncoderAPin;
	encoderBPin = EncoderBPin;

	encoderCPR = CPR;
	encoder = new Encoder(encoderAPin, encoderBPin);
}


float BrushlessMotorDriver::turnReferenceAngle() {
	uint32_t now_us = micros();
	if (lastTurnTime_us == 0) {
		// first call of this, we dont have dT
		// be aware that in this case, we return 0
		lastTurnTime_us = now_us;
	}
	uint32_t timePassed_us = now_us - lastTurnTime_us;

	// check for overflow on micros() (happens every 70 minutes at teensy's frequency of 120MHz)
	if (now_us < lastTurnTime_us) {
		logger->println("time overflow!!!");
		timePassed_us = (4294967295 - timePassed_us) + now_us;
		logger->print("timepased=");
		logger->println(timePassed_us);
	}
	lastTurnTime_us = now_us;
	float timePassed_s = (float)timePassed_us/1000000.0;
	if (timePassed_s > (2.0/(float)SampleFrequency)) {
		// this happens if this methods isnt called often enough. Mostly
		// when serial communications takes place
		logger->println("turnReferenceAngle's dT too big!!!!");
		logger->print(timePassed_s*1000.0);
		logger->println("ms");
	}

	// accelerate to target speed
	float speedDiff = targetMotorSpeed - currentReferenceMotorSpeed;

	speedDiff = constrain(speedDiff, -abs(targetAcc)*timePassed_s, +abs(targetAcc)*timePassed_s);

	currentReferenceMotorSpeed += speedDiff;
	if (timePassed_s > 0.0001) // max call frequency should be 1000Hz
		currentReferenceMotorAccel = speedDiff/timePassed_s;

	// compute angle difference compared to last invokation
	referenceAngle += currentReferenceMotorSpeed * timePassed_s * TWO_PI;

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
bool BrushlessMotorDriver::loop() {
	if (enabled) {
		// run only if at least one ms passed
		uint32_t now = millis();

		// max frequency of motor control is 1000Hz
		if (now - lastLoopCall_ms < 1000/MaxBrushlessDriverFrequency)
			return false;
		lastLoopCall_ms = now;

		// turn reference angle along the set speed
		float timePassed_s = turnReferenceAngle();
		if (timePassed_s > 0) {
			/*
			logger->print("loop t=");
			logger->print(timePassed_s,4);
			logger->print(" r=");
			logger->print(referenceAngle);
			logger->print(" e=");
			logger->print(encoderAngle);
			logger->println();
			*/
			// read the current encoder value
			readEncoder();

			// compute position error as input for PID controller
			float errorAngle = referenceAngle - encoderAngle;

			// carry out posh PID controller. Outcome is used to compute magnetic field angle (between -90° and +90°) and torque.
			// if pid's outcome is 0, magnetic field is like encoder's angle, and torque is 0
			float speedRatio = min(measuredMotorSpeed/maxRevolutionSpeed,1.0);
			float controlOutput = pid.update(memory.persistentMem.motorControllerConfig.pid_position, memory.persistentMem.motorControllerConfig.pid_speed,
											-maxAngleError /* min */,maxAngleError /* max */, speedRatio,
											errorAngle,  timePassed_s);

			// estimate the current shift of current behind voltage (back EMF). This is typically set to increase linearly with the voltage
			// which is proportional to the torque for the PWM output
			// (according to https://www.digikey.gr/en/articles/techzone/2017/jan/why-and-how-to-sinusoidally-control-three-phase-brushless-dc-motors)
			// (according to "Advance Angle Calculation for Improvement of the Torque-to Current Ratio of Brushless DC Motor Drives")
			float advanceAnglePhaseShift = (measuredMotorSpeed/maxRevolutionSpeed)*maxAdvancePhaseAngle;

			// torque is max at -90/+90 degrees
			// (https://www.roboteq.com/index.php/applications/100-how-to/359-field-oriented-control-foc-made-ultra-simple)
			advanceAngle = radians(90) * sigmoid(10.0, controlOutput);

			float torque = abs(controlOutput)/maxAngleError;

			// set magnetic field relative to rotor's position
			magneticFieldAngle = encoderAngle + advanceAngle + advanceAnglePhaseShift;

			// if the motor is not able to follow the magnetic field , limit the reference angle accordingly
			referenceAngle = constrain(referenceAngle, encoderAngle - maxAngleError, encoderAngle  + maxAngleError);
			// recompute speed, since set speed might not be achieved

			measuredMotorSpeed = (referenceAngle-lastReferenceAngle)/TWO_PI/timePassed_s;
			lastReferenceAngle = referenceAngle; // required to compute speed

			// send new pwm value to motor
			sendPWMDuty(min(abs(torque),1.0));


			/*
			static uint32_t lastoutput = 0;

			if (millis() - lastoutput >  100) {
				lastoutput = millis();
			command->print("time=");
			command->print(timePassed_s*1000.0);
			command->print("ms v=");
			command->print(currentReferenceMotorSpeed);
			command->print("/");
			command->print(actualMotorSpeed);
			command->print(" r=");
			command->print(degrees(referenceAngle));
			command->print("° enc=");
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
		*/
			return true;
		}
	}
	return false;
}

void BrushlessMotorDriver::setMotorSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	targetMotorSpeed = speed;
	targetAcc = acc;
}

float BrushlessMotorDriver::getMotorSpeed() {
	return measuredMotorSpeed;
}

float BrushlessMotorDriver::getIntegratedMotorAngle() {
	return referenceAngle;
}


void BrushlessMotorDriver::setSpeed(float speed /* rotations per second */, float acc /* rotations per second^2 */) {
	setMotorSpeed(speed/GearBoxRatio,acc);
}

float BrushlessMotorDriver::getSpeed() {
	return measuredMotorSpeed*GearBoxRatio;
}

float BrushlessMotorDriver::getIntegratedAngle() {
	return referenceAngle*GearBoxRatio;
}

void BrushlessMotorDriver::enable(bool doit) {
	enabled = doit;
	if (enabled) {
		if (memory.persistentMem.logConfig.calibrationLog) {
			logger->print("calibrating motor ");
			logger->print(motorNo);
			logger->println();
		}

		// startup procedure to find the angle of the motor's rotor
		// - turn magnetic field with min torque (120° max) until encoder recognizes a significant movement
		// - turn in other direction until this movement until encoder gives original position
		// if the encoder does not indicate a movement, increase torque and try again

		if (encoder == NULL)
			delete encoder;
		encoder = new Encoder(encoderAPin, encoderBPin);

		// enable driver, but PWM has no duty cycle yet.
		sendPWMDuty(0);
		digitalWrite(enablePin, HIGH);

		// startup calibration works by turning the magnetic field until the encoder
		// indicates the rotor being aligned with the field
		// During calibration, run a loop that
		// - measure the encoder
		// - turn the magnetic field in the direction of the deviation as indicated by the encoder
		// - if the encoders indicates no movement, increase torque
		// quit the loop if torque is above a certain threshold with encoder at 0
		// end calibration by setting the current reference angle to the measured rotors position
		int tries = 0;
		const int maxTries = 5;
		bool repeat = false;
		do {
			if ((tries > 0) && (memory.persistentMem.logConfig.calibrationLog))
				logger->println("restart ");

			lastLoopCall_ms = 0;				// time of last loop call
			measuredMotorSpeed = 0;				// [rev/s]
			referenceAngle = 0;					// [rad] target angle of the rotor
			lastReferenceAngle = 0;				// [rad]
			currentReferenceMotorSpeed = 0;		// [rev/s]
			currentReferenceMotorAccel = 0;
			measuredMotorSpeed = 0;
			targetMotorSpeed = 0;
			targetAcc = 0;
			advanceAngle = 0;
			lastTurnTime_us = 0;
			pid.reset();

			magneticFieldAngle = tries*radians(360/7);
			encoderAngle = 0;
			lastEncoderPosition = 0;
			float lastLoopEncoderAngle = 0;
			readEncoder();

			float targetTorque = 0.0;
			float lastTorque = 0;
			float maxTorque = 0.4;
			float lastTime_us = micros();
			uint32_t now_us = micros();
			float maxEncoderAngle = 0;
			float elapsedTime = 0;
			const float timeOut = 5.0;

			while ((targetTorque < maxTorque) && (elapsedTime < timeOut)) { // quit when above 80% torque or timeout of 5. happened
				now_us = micros();
				float dT = (now_us - lastTime_us)/1000000.0;
				lastTime_us = now_us;
				elapsedTime += dT;
				if ((int)(targetTorque/maxTorque*10.) > (int)(lastTorque/maxTorque*10.)) {
					logger->print(10-(int)(targetTorque/maxTorque*10.));
					logger->print("(");
					logger->print(degrees(magneticFieldAngle),0);
					logger->print("°) ");
					lastTorque = targetTorque;
				}

				// let the magnetic field turn with at least 1 rev/s towards the encoder value different from 0
				magneticFieldAngle -= sgn(encoderAngle)*min(radians(1.0),abs(encoderAngle)*0.5); // this is a P-controller that turns the magnetic field towards the direction of the encoder

				// logger->print(dT*100);
				// logger->print(" ");
				sendPWMDuty(targetTorque);
				delay(2);

				// if encoder indicates no movement, we can increase torque a bit until the motor moves
				// if there is movement, decrease the torque and let the motor turn until the rotor is
				// in line with the magnetic field
				readEncoder();
				if (abs(maxEncoderAngle) < abs(encoderAngle))
					maxEncoderAngle = encoderAngle;
				float encoderAngleDiff = encoderAngle - lastLoopEncoderAngle;
				float encoderResolution = TWO_PI/((float)encoderCPR)*2.0;
				if (abs(encoderAngleDiff) < encoderResolution && abs(encoderAngle) < encoderResolution) {
					targetTorque += dT*5.0;
					targetTorque = min(targetTorque, maxTorque);
				}

				lastLoopEncoderAngle = encoderAngle;
				if (memory.persistentMem.logConfig.calibrationLog) {
					logger->print("mag=");
					logger->print(degrees(magneticFieldAngle));
					logger->print("enc=");
					logger->print(degrees(encoderAngle));
					logger->print(" t=");
					logger->print(targetTorque);
					logger->println();
				}
			}

			// if almost no movement happened, rotor could be located in a singularity.
			// This would be bad, rotor would never find its position during rotation.
			// We should move at least 4°
			repeat  = (abs(maxEncoderAngle) < radians(4)) || (elapsedTime >= timeOut);
			if (repeat) {
				if (memory.persistentMem.logConfig.calibrationLog) {
					logger->print(" failed(");
					logger->print(degrees(maxEncoderAngle),1);
					logger->print("°,");
					logger->print(targetTorque,1);
					logger->print("PWM,");
					logger->print(elapsedTime,1);
					logger->print("s) ");
				}
			}
		}
		while ((tries++ < maxTries) && (repeat == true));
		readEncoder();
		referenceAngle = magneticFieldAngle;
		encoderAngle = magneticFieldAngle;
		lastReferenceAngle = magneticFieldAngle;

		if (tries >= maxTries) {
			digitalWrite(enablePin, HIGH);
			enabled = false;
			logger->println(" failed.");
		} else
			logger->println(" done.");
	}
	else {
		digitalWrite(enablePin, LOW);
	}
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
		bool pidChange = false;
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
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Kp  += 0.02;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Kp  += 0.02;

			pidChange = true;
			break;
		case 'p':
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Kp -= 0.02;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Kp -= 0.02;
			pidChange = true;
			break;
		case 'D':
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Kd += 0.005;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Kd += 0.005;
			pidChange = true;

			break;
		case 'd':
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Kd -= 0.005;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Kd -= 0.005;
			pidChange = true;
			break;
		case 'I':
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Ki += 0.02;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Ki += 0.02;
			pidChange = true;
			break;
		case 'i':
			if (abs(currentReferenceMotorSpeed) < 15)
				memory.persistentMem.motorControllerConfig.pid_position.Ki -= 0.02;
			else
				memory.persistentMem.motorControllerConfig.pid_speed.Ki -= 0.02;
			pidChange = true;
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

		if (pidChange) {
			command->print("PID(pos)=");
			command->print(memory.persistentMem.motorControllerConfig.pid_position.Kp);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_position.Ki);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_position.Kd);
			command->println(")");
			command->print("PID(speed)=");
			command->print(memory.persistentMem.motorControllerConfig.pid_speed.Kp);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_speed.Ki);
			command->print(",");
			command->print(memory.persistentMem.motorControllerConfig.pid_speed.Kd);
			command->println(")");
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

