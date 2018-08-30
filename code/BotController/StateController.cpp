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


void ControlPlane::update(float dT,
		float currentSpeed /* speed of body */, float targetSpeed , float targetAccel,
		float currentOmega, float targetOmega,
		float sensorAngle /* angle towards the axis */, float sensorAngularVelocity) {

	// target angle out of acceleration, assume tan(x) = x
	targetAngle = targetAccel/Gravity;

	// target angularVelocity out of acceleration
	float targetAngularVelocity = (targetAngle - lastTargetAngle)*dT;

	// compute absolute position of the ball's centre of gravity
	absBallPos += currentSpeed*dT - sensorAngularVelocity*dT/TWO_PI * CentreOfGravityHeight;

	// compute absolute position of the position where we expect the bot to be
	targetBallPos += targetSpeed*dT  - targetAngle/TWO_PI * CentreOfGravityHeight;

	// compute absolute position of the body where we expect the bot to be
	float targetBodyPos = targetBallPos + targetAngle/TWO_PI * CentreOfGravityHeight;

	// compute target speed of ball (considering the dynamic tilt angle)
	float targetBallSpeed = (targetBodyPos - lastTargetBodyPos)/dT;

	// compute ABSOLUTE position of the body, assume sin(x) = x for small angle
	float absBodyPos = absBallPos + sensorAngle*CentreOfGravityHeight;

	// compute acceleration the robot is supposed to have
	float targetAbsAccel = (targetSpeed - lastTargetSpeed)/dT;

	// compute the ABSOLUTE velocity the robot's body really has
	float absBodySpeed = (absBodyPos - lastAbsBodyPos)/dT;

	// compute the ABSOLUTE velocity the ball has
	float absBallSpeed = (absBallPos - lastAbsBallPos)/dT;

	// compute the ABSOLUTE acceleration the robot's body really has
	bodyAccel = (absBodySpeed-lastBodySpeed) / dT;

	// compute the ABSOLUTE acceleration the robot's base really has
	ballAccel = (currentSpeed-lastBallSpeed) / dT;

	// compute the error of the angle
	errorAngle = targetAngle-sensorAngle;

	// compute the error in angular velocity
	errorAngularVelocity = targetAngularVelocity-sensorAngularVelocity;

	// compute the delta of absolute as-is and to-be position of the base (positive means tobe > as-is)
	errorBallPosition = targetBallPos-absBallPos;

	// compute the delta of absolute as-is and to-be position of the body
	errorBodyPosition = targetBodyPos - absBodyPos;

	// compute the delta of the as-is and to-be speed (positive means tobe > as-is)
	errorBallVelocity = targetBallSpeed - absBallSpeed;

	// compute the delta of the as-is and to-be body speed (positive means tobe > as-is)
	errorBodyVelocity = targetSpeed-absBodySpeed;

	// compute the difference between the as-is and to-be body acceleration (positive means tobe > as-is)
	errorBodyAccel = (targetAccel - bodyAccel);

	// compute the difference between the as-is and to-be body acceleration (positive means tobe > as-is)
	errorBallAccel = targetAbsAccel - ballAccel;

	// compute error against centripedal force, which is f=omega*v*m*c, where m*c is the weight
	float errorCentripedal = targetOmega * targetSpeed;

	// now multiply all deltas in each state variable with the according weight
	float error_tilt			= memory.persistentMem.ctrlConfig.angleWeight * errorAngle;
	float error_angular_speed	= memory.persistentMem.ctrlConfig.angularSpeedWeight * sensorAngularVelocity;

	float error_base_position 	= memory.persistentMem.ctrlConfig.ballPositionWeight * errorBallPosition;
	float error_base_velocity 	= memory.persistentMem.ctrlConfig.ballVelocityWeight * errorBallVelocity;
	float error_base_accel		= memory.persistentMem.ctrlConfig.ballAccelWeight * errorBallAccel;

	float error_body_position	= memory.persistentMem.ctrlConfig.bodyPositionWeight * errorBodyPosition;
	float error_body_velocity	= memory.persistentMem.ctrlConfig.bodyVelocityWeight * errorBodyVelocity;
	float error_body_accel		= memory.persistentMem.ctrlConfig.bodyAccelWeight * errorBodyAccel;

	float error_centripedal     = memory.persistentMem.ctrlConfig.omegaWeight* errorCentripedal;

	// sum up all weighted errors
	float error =       -error_tilt - error_angular_speed
						-error_base_velocity - error_base_position - error_base_accel
						-error_body_velocity - error_body_position - error_body_accel
						-error_centripedal;

	accel = constrain(error,-MaxBotAccel, MaxBotAccel);

	// accelerate only if not yet on max speed
	if ((sgn(speed) != sgn(accel)) ||
		(abs(speed) < MaxBotSpeed))
		speed += accel * dT;

	lastTargetAngle = targetAngle;
	lastAbsBodyPos = absBodyPos;
	lastAbsBallPos = absBallPos;
	lastTargetBodyPos = targetBodyPos;
	lastBodySpeed = absBodySpeed;
	lastBallSpeed = absBallSpeed;
	lastTargetSpeed = targetSpeed;

	// in order to increase gain of state controller, filter with FIR 20Hz 4th order
	filteredSpeed = speedFilter.update(speed);
}

void StateController::setup(MenuController* menuCtrl) {
	registerMenuController(menuCtrl);
}


void StateController::update(float dT, const BotMovement& currentMovement,
							 const IMUSample& sensorSample,
							 const BotMovement& targetBotMovement) {
	// ramp up target speed and omega with a trapezoid profile of constant acceleration
	rampedTargetMovement.rampUp(targetBotMovement, dT);

	planeX.update(dT, currentMovement.speedX, rampedTargetMovement.speedX, rampedTargetMovement.accelX, currentMovement.omega, rampedTargetMovement.omega, sensorSample.y.angle, sensorSample.y.angularVelocity);
	planeY.update(dT, currentMovement.speedY, rampedTargetMovement.speedY, rampedTargetMovement.accelY, currentMovement.omega, rampedTargetMovement.omega, sensorSample.y.angle, sensorSample.x.angularVelocity);

	// omega is not filtered
}

float StateController::getSpeedX() {
	return planeX.filteredSpeed;
}
float StateController::getSpeedY() {
	return planeX.filteredSpeed;
}

float StateController::getOmega() {
	return rampedTargetMovement.omega;
}

void StateController::printHelp() {
	command->println();
	command->println("State controller");
	command->println();
	command->println("ESC");
}

void StateController::menuLoop(char ch) {

		bool cmd = true;
		switch (ch) {
		case 'h':
			printHelp();
			break;
		default:
			cmd = false;
			break;
		}
		if (cmd) {
			command->print(">");
		}
}

