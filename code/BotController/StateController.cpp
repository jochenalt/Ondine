/*
 * StateController.cpp
 *
 *  Created on: 23.08.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"
#include <BotMemory.h>
#include <Util.h>

#include <MenuController.h>
#include <StateController.h>
#include <BotController.h>


void ControlPlane::reset () {
			lastTargetAngle = 0;
			lastBodyPos = 0; // absolute body position of last loop
			lastBallPos = 0;
			lastBodySpeed = 0;
			lastBallSpeed = 0;
			lastTargetBodyPos = 0;
			lastTargetBallPos = 0;
			lastTargetBallSpeed = 0;
			lastTargetBodySpeed = 0;
			speed = 0;
			accel  = 0;
			filteredSpeed = 0;

			// add an FIR Filter with 15Hz to the output of the controller in order to increase gain of state controller
			outputSpeedFilter.init(FIR::LOWPASS,
					         1.0e-3f  			/* allowed ripple in passband in amplitude is 0.1% */,
							 1.0e-6 			/* supression in stop band is -60db */,
							 SampleFrequency, 	/* 200 Hz */
							 20.0f  			/* low pass cut off frequency */);

			inputBallAccel.init(FIR::LOWPASS,
					         1.0e-3f  			/* allowed ripple in passband in amplitude is 0.1% */,
							 1.0e-6 			/* supression in stop band is -60db */,
							 SampleFrequency, 	/* 200 Hz */
							 50.0f  			/* low pass cut off frequency */);

			inputBodyAccel.init(FIR::LOWPASS,
					          1.0e-3f  			/* allowed ripple in passband in amplitude is 0.1% */,
							  1.0e-6 			/* supression in stop band is -60db */,
							  SampleFrequency,  /* 200 Hz */
							  50.0f  			/* low pass cut off frequency */);
}

float ControlPlane::getBodyPos() {
	return lastBodyPos;
}

float ControlPlane::getBallPos() {
	return lastBallPos;
}


void ControlPlane::update(bool log, float dT,
		const State& current, const State& target,
		float currentOmega, float targetOmega,
		const IMUSamplePlane &sensor) {

	if (dT) {
		float maxTiltError     			= MaxTiltAngle;
		float maxAngularVelocityError   = MaxTiltAngle/dT;

		const float maxPositionError = memory.persistentMem.ctrlConfig.angleWeight * maxTiltError;
		const float maxSpeedError    = maxPositionError / dT;
		const float maxAccelError    = maxSpeedError / dT;

		// target angle out of acceleration, assume tan(x) = x
		float targetAngle = target.accel/Gravity;

		// target angularVelocity out of acceleration
		float targetAngularVelocity = (targetAngle - lastTargetAngle)*dT;

		// compute pos,speed,accel of ball and body
		float absBallPos   		= current.pos;
		float absBallSpeed 		= current.speed;
		float absBallAccel 		= current.accel;

		float absBodyPos   		= current.pos + sensor.angle * CentreOfGravityHeight;
		float absBodySpeed 		= (absBodyPos - lastBodyPos)/dT;
		float absBodyAccel 		= (absBodySpeed - lastBodySpeed) / dT;

		// compute position,speed,accel where we expect the bot to be
		float targetBodyPos 	= target.pos;
		float targetBodySpeed	= target.speed;
		float targetBodyAccel	= (target.speed - lastTargetBodySpeed)/dT;

		float targetBallPos	 	= target.pos - targetAngle * CentreOfGravityHeight;
		float targetBallSpeed 	= (targetBallPos - lastTargetBallPos)/dT;
		float targetBallAccel 	= (targetBallAccel - lastTargetBallSpeed)/dT;

		// now multiply all deltas in each state variable with the according weight
		float error_tilt			= memory.persistentMem.ctrlConfig.angleWeight 		 * (sensor.angle-targetAngle);
		float error_angular_speed	= memory.persistentMem.ctrlConfig.angularSpeedWeight * (sensor.angularVelocity-targetAngularVelocity);

		float error_ball_position 	= memory.persistentMem.ctrlConfig.ballPositionWeight * (absBallPos 		-	targetBallPos);
		float error_ball_velocity 	= memory.persistentMem.ctrlConfig.ballVelocityWeight * (absBallSpeed	- 	targetBallSpeed); // [0]
		float error_ball_accel		= memory.persistentMem.ctrlConfig.ballAccelWeight 	 * (absBallAccel	-	targetBallAccel);

		float error_body_position	= memory.persistentMem.ctrlConfig.bodyPositionWeight * (absBodyPos		-	targetBodyPos); // [0]
		float error_body_velocity	= memory.persistentMem.ctrlConfig.bodyVelocityWeight * (absBodySpeed	-	targetBodySpeed);
		float error_body_accel		= memory.persistentMem.ctrlConfig.bodyAccelWeight    * (absBodyAccel	-	targetBodyAccel);    // [0]

		// compute error against centripedal force, which is f=omega*v*m*c, where m*c is the weight
		float error_centripedal     = memory.persistentMem.ctrlConfig.omegaWeight        * targetOmega * target.speed;

		error_ball_position = constrain(error_ball_position, -maxPositionError, maxPositionError);
		error_ball_accel = constrain(error_ball_accel, -maxAccelError, maxAccelError);
		error_body_position = constrain(error_body_position, -maxPositionError, maxPositionError);
		error_body_accel = constrain(error_body_accel, -maxAccelError, maxAccelError);

		// sum up all weighted errors
		float error =	-error_tilt - error_angular_speed -
						-error_ball_velocity - error_ball_position - error_ball_accel +
						-error_body_velocity - error_body_position - error_body_accel +
						-error_centripedal;

		if (log) {
			if (memory.persistentMem.logConfig.debugStateLog) {
				logger->print("currV=");
				logger->print(current.speed);
				logger->print(" currA=");
				logger->print(sensor.angle);
				logger->print(" currA'=");
				logger->print(sensor.angularVelocity);
				logger->print(" targA=");
				logger->print(targetAngle);

				logger->print(" body=(");
				logger->print(absBodyPos);
				logger->print(",");
				logger->print(absBodySpeed);
				logger->print(",");
				logger->print(absBallAccel);
				logger->print(") ball=(");
				logger->print(absBallPos);
				logger->print(",");
				logger->print(absBallSpeed);
				logger->print(",");
				logger->print(absBallAccel);
				logger->print(")");

				logger->print("error=(");
				logger->print(error_tilt);
				logger->print(",");
				logger->print(error_angular_speed);
				logger->print("|");
				logger->print(error_ball_position);
				logger->print(",");
				logger->print(error_ball_velocity);
				logger->print(",");
				logger->print(error_ball_accel);
				logger->print("|");
				logger->print(error_body_position);
				logger->print(",");
				logger->print(error_body_velocity);
				logger->print(",");
				logger->print(error_body_accel);
				logger->print("|=");
				logger->print(error);
				logger->print(")");
			}
		}
		accel = constrain(error,-MaxBotAccel, MaxBotAccel);

		// accelerate if not on max speed already
		if ((sgn(speed) != sgn(accel)) ||
			(abs(speed) < MaxBotSpeed)) {
			speed += accel * dT;
			speed = constrain(speed, -MaxBotSpeed, + MaxBotSpeed);
		}

		lastTargetAngle = targetAngle;
		lastBodyPos = absBodyPos;
		lastBodySpeed = absBodySpeed;

		lastBallPos = absBallPos;
		lastBallSpeed = absBallSpeed;

		lastTargetBodyPos = targetBodyPos;
		lastTargetBodySpeed = targetBodySpeed;

		lastTargetBallPos = targetBallPos;
		lastTargetBallSpeed = targetBallSpeed;


		// in order to increase gain of state controller, filter with FIR 20Hz 4th order
		filteredSpeed = outputSpeedFilter.update(speed);
		if (log)
			if (memory.persistentMem.logConfig.debugStateLog) {
				logger->print(" output=(");
				logger->print(accel);
				logger->print(",");
				logger->print(speed);
				logger->print(",");
				logger->print(filteredSpeed);
				logger->print(")");
			}
	};
}

void StateController::setup(MenuController* menuCtrl) {
	registerMenuController(menuCtrl);
	reset();
}

void StateController::reset() {
	planeX.reset();
	planeY.reset();
	rampedTargetMovement.reset();
}

void StateController::update(float dT,
							 const IMUSample& sensorSample,
							 const BotMovement& currentMovement,
							 const BotMovement& targetBotMovement) {

	uint32_t start = millis();
	// ramp up target speed and omega with a trapezoid profile of constant acceleration
	rampedTargetMovement.rampUp(targetBotMovement, dT);
	bool log = logTimer.isDue_ms(1000,millis());
	if (log && memory.persistentMem.logConfig.debugStateLog)
		logger->print("   planeX:");
	planeX.update(log, dT,
					currentMovement.x, rampedTargetMovement.x,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::X]);

	if (log && memory.persistentMem.logConfig.debugStateLog) {
		logger->println();
		logger->print("   planeY:");
	}
	planeY.update(log, dT,
					currentMovement.y, rampedTargetMovement.y,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::Y]);
	if (log && memory.persistentMem.logConfig.debugStateLog) {
		logger->println();
	}
	uint32_t end = millis();
	avrLoopTime = (avrLoopTime + ((float)(end - start)*0.001))*0.5;
}

float StateController::getSpeedX() {
	return planeX.filteredSpeed;
}
float StateController::getSpeedY() {
	return planeY.filteredSpeed;
}

float StateController::getOmega() {
	return rampedTargetMovement.omega;
}

void StateController::printHelp() {
	command->println();
	command->println("State controller");
	command->println();

	command->println("q/Q - angle weight");
	command->println("a/A - angular speed weight");
	command->println("w/W - ball position weight");
	command->println("s/S - ball speed weight");
	command->println("r/r - ball accel weight");
	command->println("f/f - body position weight");
	command->println("t/T - body speed weight");
	command->println("g/G - body accel weight");
	command->println("z/Z - omega weight");
	command->println("b   - balance on/off");

	command->println("0   - set null");

	command->println();
	command->println("ESC");
}


void StateController::menuLoop(char ch, bool continously) {

		bool cmd = true;
		switch (ch) {
		case 'h':
			printHelp();
			break;
		case 'b':
			BotController::getInstance().balanceMode(BotController::getInstance().isBalancing()?
											BotController::BotMode::OFF:
											BotController::BotMode::BALANCING);
			if (BotController::getInstance().isBalancing())
				logger->println("balancing mode on");
			else
				logger->println("balancing mode off");
			break;
		case '0':
			memory.persistentMem.ctrlConfig.angleWeight = 0.0;
			memory.persistentMem.ctrlConfig.angularSpeedWeight = 0.0;
			memory.persistentMem.ctrlConfig.ballPositionWeight = 0.0;
			memory.persistentMem.ctrlConfig.ballVelocityWeight = 0.;
			memory.persistentMem.ctrlConfig.ballAccelWeight = 0.0;
			memory.persistentMem.ctrlConfig.bodyPositionWeight = 0.;
			memory.persistentMem.ctrlConfig.bodyVelocityWeight = 0.0;
			memory.persistentMem.ctrlConfig.bodyAccelWeight = 0.;
			memory.persistentMem.ctrlConfig.omegaWeight = 0.;
			break;
		case 'q':
			memory.persistentMem.ctrlConfig.angleWeight -= continously?2.0:0.5;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'Q':
			memory.persistentMem.ctrlConfig.angleWeight += continously?2.0:0.5;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'a':
			memory.persistentMem.ctrlConfig.angularSpeedWeight -= continously?2.0:0.5;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'A':
			memory.persistentMem.ctrlConfig.angularSpeedWeight += continously?2.0:0.5;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'w':
			memory.persistentMem.ctrlConfig.ballPositionWeight -= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'W':
			memory.persistentMem.ctrlConfig.ballPositionWeight += continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'y':
			memory.persistentMem.ctrlConfig.ballVelocityWeight-= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'Y':
			memory.persistentMem.ctrlConfig.ballVelocityWeight += continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'r':
			memory.persistentMem.ctrlConfig.ballAccelWeight-= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'R':
			memory.persistentMem.ctrlConfig.ballAccelWeight += continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'f':
			memory.persistentMem.ctrlConfig.bodyPositionWeight +=continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'F':
			memory.persistentMem.ctrlConfig.bodyPositionWeight-= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 't':
			memory.persistentMem.ctrlConfig.bodyVelocityWeight += continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();

			cmd = true;
			break;
		case 'T':
			memory.persistentMem.ctrlConfig.bodyVelocityWeight -= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd =true;
			break;
		case 'g':
			memory.persistentMem.ctrlConfig.bodyAccelWeight += continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;
		case 'G':
			memory.persistentMem.ctrlConfig.bodyAccelWeight -= continously?0.05:0.01;
			memory.persistentMem.ctrlConfig.print();
			cmd = true;
			break;

		default:
			cmd = false;
			break;
		}
		if (cmd) {
			command->print(">");
		}
}

