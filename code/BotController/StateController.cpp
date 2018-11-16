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


void StateControllerConfig::print() {
	StateControllerConfig defValue;
	defValue.initDefaultValues();
	logln("state controller configuration:");
	log("   angle=");
	log(angleWeight,4,0);
	log("(");
	log(defValue.angleWeight);
	log(")");
	log(" angularSpeed=");
	log(angularSpeedWeight,4,0);
	log("(");
	logln(defValue.angularSpeedWeight);
	log("   intBallPos=");
	log(ballPosIntegratedWeight,4,0);
	log("(");
	log(defValue.ballPosIntegratedWeight,4,0);
	log(")");
	log(" ballPos=");
	log(ballPositionWeight,4,0);
	log("(");
	log(defValue.ballPositionWeight,4,0);
	log(")");
	log(" ballSpeed=");
	log(ballVelocityWeight,4,0);
	log("(");
	log(defValue.ballVelocityWeight,4,0);
	log(")");
	log("   omega=");
	log(omegaWeight,4,0);
	log("(");
	log(defValue.omegaWeight,4,0);
	logln(")");
}

void StateControllerConfig::initDefaultValues() {

	// initialize the weights used for the state controller per
	// basic values can be tried out  via https://robotic-controls.com/learn/inverted-pendulum-controls
	// with mc = 1.2 kg, mb = 0.1 kg, L = 0.15
	angleWeight				= 200.0; // 39.0;
	angularSpeedWeight		= 10.0; // 21.00;

	ballPosIntegratedWeight = 60.;
	ballPositionWeight		= 10.;
	ballVelocityWeight		= 30.0;

	omegaWeight				= 0.0;
}


void ControlPlane::reset () {
			lastTargetAngle = 0;
			lastBallPos = 0;
			lastBallSpeed = 0;
			lastTargetBodyPos = 0;
			lastTargetBallPos = 0;
			lastTargetBallSpeed = 0;
			filteredSpeed = 0;
			speed = 0;
			ballPosIntegrated = 0;

			// add an FIR Filter with 15Hz to the output of the controller in order to increase gain of state controller
			outputSpeedFilter.init(FIR::LOWPASS,
					         1.0e-3f  			/* allowed ripple in passband in amplitude is 0.1% */,
							 1.0e-6 			/* supression in stop band is -60db */,
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

		// target angularVelocity out of acceleration
		float targetAngularVelocity = (targetAngle - lastTargetAngle)*dT;

		// compute pos,speed,accel of the ball
		float absBallPos   		= current.pos;
		float absBallSpeed 		= current.speed;

		// compute target position,speed, and accel
		float targetBallPos	 	= target.pos - targetAngle * CentreOfGravityHeight;
		float targetBallSpeed 	= (targetBallPos - lastTargetBallPos)/dT;
		float targetBallAccel 	= (targetBallAccel - lastTargetBallSpeed)/dT;	// does not need to be filtered, it is generated in a smooth way already

		// errors
		float error_tilt			= (sensor.angle-targetAngle)/MaxTiltAngle;
		float error_angular_speed	= (sensor.angularVelocity-targetAngularVelocity)/MaxTiltAngle;

		float error_ball_position 	= (absBallPos 	- targetBallPos);
		ballPosIntegrated 			+= error_ball_position*dT;
		float error_ball_velocity 	= (absBallSpeed	- targetBallSpeed);

		float error_centripedal     = targetOmega * target.speed;
		StateControllerConfig& config = memory.persistentMem.ctrlConfig;
		/*
		if (abs(config.ballPositionWeight) > 0.01)
			error_ball_position = constrain(error_ball_position,  -config.angleWeight*MaxTiltAngle/config.ballPositionWeight, -config.angleWeight*MaxTiltAngle/config.ballPositionWeight);
		*/
		// sum up all weighted errors
		float error =	- config.angleWeight*error_tilt - config.angularSpeedWeight*error_angular_speed
						+ config.ballPosIntegratedWeight*ballPosIntegrated + config.ballPositionWeight*error_ball_position + config.ballVelocityWeight*error_ball_velocity
						+ config.omegaWeight * error_centripedal;

		if (doLogging) {
			if (memory.persistentMem.logConfig.debugStateLog) {
				log("v=");
				log(current.speed,2,3);
				log(" a=");
				log(sensor.angle,2,3);
				log(" w=");
				log(sensor.angularVelocity,2,3);
				log(" x=");
				log(absBallPos,2,3);
				log(" v=");
				log(absBallSpeed,2,3);

				log(" error=");
				log(error_tilt,3,3);
				log(",");
				log(error_angular_speed,3,3);
				log("|");
				log(ballPosIntegrated,3,3);
				log(",");
				log(error_ball_position,3,3);
				log(",");
				log(error_ball_velocity,3,3);
				log("|=");
				log(error,3,3);
				log(")");
			}
		}

		// outcome of controller is force to be applied at the ball
		float force = constrain(error,-MaxBotAccel, MaxBotAccel);

		// F = m*a
		float accel = force / BallWeight;

		// accelerate if not on max speed already
		if ((sgn(speed) != sgn(accel)) ||
			(abs(speed) < MaxBotSpeed)) {
			speed += accel * dT;
			speed = constrain(speed, -MaxBotSpeed, + MaxBotSpeed);
		}

		// get rid of trembling by a FIR filter 4th order with 15Hz
		filteredSpeed = outputSpeedFilter.update(speed);

		lastTargetAngle = targetAngle;

		lastBallPos = absBallPos;
		lastBallSpeed = absBallSpeed;
		lastTargetBallPos = targetBallPos;
		lastTargetBallSpeed = targetBallSpeed;


		if (doLogging)
			if (memory.persistentMem.logConfig.debugStateLog) {
				log(" output=(");
				log(accel,3,3);
				log(",");
				log(speed,3,3);
				log(",");
				log(filteredSpeed,3,3);
				log(")");
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
		log("   planeX:");
	planeX.update(doLogging, dT,
					currentMovement.x, rampedTargetMovement.x,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::X]);

	if (doLogging && memory.persistentMem.logConfig.debugStateLog) {
		logln();
		log("   planeY:");
	}
	planeY.update(doLogging, dT,
					currentMovement.y, rampedTargetMovement.y,
					currentMovement.omega, rampedTargetMovement.omega,
					sensorSample.plane[Dimension::Y]);
	if (doLogging && memory.persistentMem.logConfig.debugStateLog) {
		logln();
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
	command->println("w/W - angular speed weight");
	command->println();
	command->println("A/a - ball position weight");
	command->println("S/s - ball pos integrated weight");
	command->println("D/d - ball speed weight");
	command->println();
	command->println("z/Z - omega weight");
	command->println("b   - balance on/off");

	command->println("0   - set null");

	command->println();
	command->println("ESC");
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
				logln("balancing mode on");
			else
				logln("balancing mode off");
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
			config.angleWeight -= continously?2.0:0.5;
			config.print();
			cmd =true;
			break;
		case 'Q':
			config.angleWeight += continously?2.0:0.5;
			config.print();
			cmd = true;
			break;
		case 'w':
			config.angularSpeedWeight -= continously?1.0:0.2;
			config.print();
			cmd =true;
			break;
		case 'W':
			config.angularSpeedWeight += continously?1.0:0.2;
			config.print();
			cmd = true;
			break;
		case 'a':
			config.ballPositionWeight -= continously?1.00:0.2;
			config.print();
			cmd =true;
			break;
		case 'A':
			config.ballPositionWeight += continously?1.00:0.2;
			config.print();
			cmd = true;
			break;
		case 's':
			config.ballPosIntegratedWeight-= continously?1.0:0.2;
			config.print();
			cmd = true;
			break;
		case 'S':
			config.ballPosIntegratedWeight += continously?1.0:0.2;
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
			command->print(">");
		}
}

