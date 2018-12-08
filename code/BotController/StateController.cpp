/*
 * StateController.cpp
 *
 *  Created on: 23.08.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"
#include <BotMemory.h>
#include <libraries/Util.h>
#include <Setup.h>

#include <libraries/MenuController.h>
#include <StateController.h>
#include <BotController.h>


void StateControllerConfig::print() {
	StateControllerConfig defValue;
	defValue.initDefaultValues();
	loggingln("state controller:");
	logging("   PD(angle)=(");
	logging(angleWeight,2,2);
	logging("[");
	logging(defValue.angleWeight,2,2);
	logging("]");
	logging(",");
	logging(angularSpeedWeight,2,2);
	logging("[");
	logging(defValue.angularSpeedWeight,2,2);
	loggingln("])");
	logging("   PID(pos) =(");
	logging(ballPositionWeight,2,2);
	logging("[");
	logging(defValue.ballPositionWeight,2,2);
	logging("]");
	logging(",");
	logging( ballPosIntegratedWeight,2,2);
	logging("[");
	logging(defValue.ballPosIntegratedWeight,2,2);
	logging("]");
	logging(",");
	logging(ballVelocityWeight,2,2);
	logging("[");
	logging(defValue.ballVelocityWeight,2,2);
	loggingln("])");
	logging("   P(omega) =(");
	logging(omegaWeight,2,2);
	logging("[");
	logging(defValue.omegaWeight,2,2);
	loggingln("])");
}

void StateControllerConfig::initDefaultValues() {

	// initialize the weights used for the state controller per
	// basic values can be tried out  via https://robotic-controls.com/learn/inverted-pendulum-controls
	// with mc = 1.2 kg, mb = 0.1 kg, L = 0.15
	angleWeight				= 2.6; // 39.0;
	angularSpeedWeight		= 1.7; // 21.00;

	ballPositionWeight		= 0.3; // -14.2;
	ballPosIntegratedWeight = 0.25; // -0.0;
	ballVelocityWeight		= 0.2; // -12.0;

	omegaWeight				= 0.0;
}


void ControlPlane::reset () {
			lastTargetAngle = 0;
			lastAngle = 0;
			lastBallPos = 0;
			lastBallSpeed = 0;
			lastTargetBodyPos = 0;
			lastTargetBallPos = 0;
			filteredSpeed = 0;
			speed = 0;
			posErrorIntegrated = 0;

			// add an FIR Filter with 15Hz to the output of the controller in order to increase gain of state controller
			outputSpeedFilter.init(FIR::LOWPASS,
					         1.0e-4f  			/* allowed ripple in passband in amplitude is 0.1% */,
							 1.0e-7 			/* supression in stop band is -60db */,
							 SampleFrequency, 	/* 200 Hz */
							 15.0f  			/* low pass cut off frequency */);
}

float ControlPlane::getBodyPos() {
	return lastBallPos + lastTargetAngle*CentreOfGravityHeight;
}

float ControlPlane::getBallPos() {
	return lastBallPos;
}


void ControlPlane::update(bool doLogging, float dT,
		const State& current, const State& target,
		float currentOmega, float targetOmega,
		const IMUSamplePlane &sensor) {

	// control algoritm
	// 		θ = tilt angle in [rad]
	// 		ω = dθ / dt  [rad/s]
	// 		x = ball position [m]
	// 		v = dx/dt [m/s]
	// F = -kPθ·θ - kDθ·ω + kPx·x + kIx·∫xdt + kDx·v

	if (dT) {
		// target angle out of acceleration, assume tan(x) = x
		float targetAngle = target.accel/Gravity;

		// compute current state variables angle, angular velocity, position, speed, accelst arget angularVelocity out of acceleration
		float targetAngularVelocity = (targetAngle - lastTargetAngle)*dT;
		float absBallPos   		= current.pos;
		float absBallSpeed 		= current.speed;
		float targetBallPos	 	= target.pos - targetAngle * CentreOfGravityHeight;
		float targetBallSpeed 	= (targetBallPos - lastTargetBallPos)/dT;

		// compute erros for PD(angle) and PID(position)
		float error_tilt			= (sensor.angle-targetAngle);
		float error_angular_speed	= (sensor.angularVelocity-targetAngularVelocity);

		float posError 	= (absBallPos + sensor.angle * CentreOfGravityHeight*0 - targetBallPos);
		posErrorIntegrated 			+= posError*dT;
		posErrorIntegrated 			= constrain(posErrorIntegrated, -MaxTiltAngle, MaxTiltAngle);
		float posVelocityError 		= (absBallSpeed	- targetBallSpeed);

		float error_centripedal     = targetOmega * target.speed;
		StateControllerConfig& config = memory.persistentMem.ctrlConfig;
		/*
		if (abs(config.ballPositionWeight) > 0.01)
			error_ball_position = constrain(error_ball_position,  -config.angleWeight*MaxTiltAngle/config.ballPositionWeight, -config.angleWeight*MaxTiltAngle/config.ballPositionWeight);
		*/
		// sum up all weighted errors
		float error =	+ config.angleWeight*error_tilt + config.angularSpeedWeight*error_angular_speed
						+ config.ballPositionWeight*posError + config.ballPosIntegratedWeight*posErrorIntegrated  + config.ballVelocityWeight*posVelocityError
						+ config.omegaWeight * error_centripedal;

		if (doLogging) {
			if (memory.persistentMem.logConfig.debugStateLog) {
				logging("v=");
				logging(current.speed,2,3);
				logging(" a=");
				logging(sensor.angle,2,3);
				logging(" w=");
				logging(sensor.angularVelocity,2,3);
				logging(" x=");
				logging(absBallPos,2,3);
				logging(" v=");
				logging(absBallSpeed,2,3);

				logging(" error=");
				logging(error_tilt,3,3);
				logging(",");
				logging(error_angular_speed,3,3);
				logging("|");
				logging(posErrorIntegrated,3,3);
				logging(",");
				logging(posError,3,3);
				logging(",");
				logging(posVelocityError,3,3);
				logging("|=");
				logging(error,3,3);
				logging(")");
			}
		}

		// outcome of controller is force to be applied to the ball
		// F = m*a,
		float force = error;
		float accel = force / BallWeight;

		accel = constrain(accel,-MaxBotAccel, MaxBotAccel);

		// accelerate if not on max speed already
		if ((sgn(speed) != sgn(accel)) ||
			(abs(speed) < MaxBotSpeed)) {
			speed += accel * dT;
			speed = constrain(speed, -MaxBotSpeed/SamplingTime, + MaxBotSpeed/SamplingTime);
		}

		// get rid of trembling by a FIR filter 4th order with 15Hz
		filteredSpeed = outputSpeedFilter.update(speed);

		lastTargetAngle = targetAngle;
		lastAngle = sensor.angle;

		lastBallPos = absBallPos;
		lastBallSpeed = absBallSpeed;
		lastTargetBallPos = targetBallPos;

		if (doLogging)
			if (memory.persistentMem.logConfig.debugStateLog) {
				logging(" output=(");
				logging(accel,3,3);
				logging(",");
				logging(speed,3,3);
				logging(",");
				logging(filteredSpeed,3,3);
				logging(")");
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
	bool doLogging = logTimer.isDue_ms(1000,millis());
	if (doLogging && memory.persistentMem.logConfig.debugStateLog)
		logging("   planeX:");
	planeX.update(doLogging, dT,
					currentMovement.x, rampedTargetMovement.x,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::X]);

	if (doLogging && memory.persistentMem.logConfig.debugStateLog) {
		loggingln();
		logging("   planeY:");
	}
	planeY.update(doLogging, dT,
					currentMovement.y, rampedTargetMovement.y,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::Y]);
	if (doLogging && memory.persistentMem.logConfig.debugStateLog) {
		loggingln();
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
	loggingln();
	loggingln("State controller");
	loggingln();

	loggingln("q/Q - angle weight");
	loggingln("w/W - angular speed weight");
	loggingln();
	loggingln("A/a - ball position weight");
	loggingln("S/s - ball pos integrated weight");
	loggingln("D/d - ball speed weight");
	loggingln();
	loggingln("z/Z - omega weight");
	loggingln("b   - balance on/off");

	loggingln("0   - set null");
	loggingln();
	loggingln("ESC");
	loggingln();
	memory.persistentMem.ctrlConfig.print();
	loggingln();
}


void StateController::menuLoop(char ch, bool continously) {

		bool cmd = true;
		StateControllerConfig& config = memory.persistentMem.ctrlConfig;
		switch (ch) {
		case 'h':
			printHelp();
			break;
		case 'b':
			BotController::getInstance().balanceMode(BotController::getInstance().isBalancing()?
											BotController::BotMode::OFF:
											BotController::BotMode::BALANCING);
			if (BotController::getInstance().isBalancing())
				loggingln("balancing mode on");
			else
				loggingln("balancing mode off");
			break;
		case '0':
			config.angleWeight = 0.0;
			config.angularSpeedWeight = 0.0;
			config.ballPosIntegratedWeight = 0.0;
			config.ballPositionWeight = 0.0;
			config.ballVelocityWeight = 0.;
			config.omegaWeight = 0.;
			break;
		case 'q':
			config.angleWeight -= continously?2.0:0.2;
			config.print();
			cmd =true;
			break;
		case 'Q':
			config.angleWeight += continously?2.0:0.2;
			config.print();
			cmd = true;
			break;
		case 'w':
			config.angularSpeedWeight -= continously?1.0:0.1;
			config.print();
			cmd =true;
			break;
		case 'W':
			config.angularSpeedWeight += continously?1.0:0.1;
			config.print();
			cmd = true;
			break;
		case 'a':
			config.ballPositionWeight -= continously?1.00:0.1;
			config.print();
			cmd =true;
			break;
		case 'A':
			config.ballPositionWeight += continously?1.00:0.1;
			config.print();
			cmd = true;
			break;
		case 's':
			config.ballPosIntegratedWeight-= continously?0.5:0.05;
			config.print();
			cmd = true;
			break;
		case 'S':
			config.ballPosIntegratedWeight += continously?0.5:0.05;
			config.print();
			cmd = true;
			break;
		case 'd':
			config.ballVelocityWeight-= continously?0.5:0.1;
			config.print();
			cmd =true;
			break;
		case 'D':
			config.ballVelocityWeight += continously?0.5:0.1;
			config.print();
			cmd = true;
			break;
		default:
			cmd = false;
			break;
		}
		if (cmd) {
			logging(">");
		}
}

