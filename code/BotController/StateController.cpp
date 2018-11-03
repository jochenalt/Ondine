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


void ControlPlane::init () {
			absBallPos = 0;		// absolute position of the base (origin = position when the bot has been switched on)
			lastTargetAngle = 0;
			lastAbsBodyPos = 0; // absolute body position of last loop
			lastAbsBallPos = 0;
			lastBodySpeed = 0;
			lastBallSpeed = 0;
			lastTargetBodyPos = 0;
			lastTargetBallPos = 0;
			lastTargetSpeed = 0;
			targetBallPos = 0; // to-be position of the base
			bodyVelocity = 0,	// absolute velocity the bot's body has
			speed = 0;
			targetAngle = 0;
			errorAngle = 0;
			errorAngularVelocity= 0;
			errorBallPosition = 0;
			errorBodyPosition = 0;
			errorBallVelocity = 0;
			errorBodyVelocity = 0;
			accel  = 0;
			filteredSpeed = 0;

			// add an FIR Filter with 15Hz to the output of the controller in order to increase gain of preveious state controller
			outputSpeedFilter.init(FIR::LOWPASS,
					         0.01f  			/* allowed ripple in passband in amplitude is 1% */,
							 1.0e-6 			/* supression in stop band is 60db */,
							 SampleFrequency, 	/* 100 Hz */
							 15.0f  			/* low pass cut off frequency */);

			inputBallAccel.init(FIR::LOWPASS,
					         0.1f  			    /* allowed ripple in passband in amplitude is 10% */,
							 1.0e-6 			/* supression in stop band is 60db */,
							 SampleFrequency, 	/* 100 Hz */
							 50.0f  			/* low pass cut off frequency */);

			inputBodyAccel.init(FIR::LOWPASS,
							  0.1f  			/* allowed ripple in passband in amplitude is 10% */,
							  1.0e-6 			/* supression in stop band is 60db */,
							  SampleFrequency,  /* 100 Hz */
							  50.0f  			/* low pass cut off frequency */);
}

void ControlPlane::update(float dT,
		float currentSpeed /* speed of body */, float targetSpeed, float targetAccel,
		float currentOmega, float targetOmega,
		float sensorAngle /* angle towards the axis */, float sensorAngularVelocity) {

	if (dT) {
		// target angle out of acceleration, assume tan(x) = x
		targetAngle = targetAccel/Gravity;

		if (memory.persistentMem.logConfig.debugBalanceLog) {
			logger->print("currV=");
			logger->print(currentSpeed);
			logger->print(" currA=");
			logger->print(sensorAngle);
			logger->print(" currA'=");
			logger->print(sensorAngularVelocity);
			logger->print(" targA=");
			logger->print(targetAngle);
		}


		// target angularVelocity out of acceleration
		float targetAngularVelocity = (targetAngle - lastTargetAngle)*dT;

		// compute absolute position of the ball's centre of gravity
		absBallPos += currentSpeed*dT - sensorAngularVelocity*dT/TWO_PI * CentreOfGravityHeight;

		// compute absolute position of the position where we expect the bot to be
		targetBallPos += targetSpeed*dT  - targetAngle/TWO_PI * CentreOfGravityHeight;

		// compute absolute position of the body where we expect the bot to be
		float targetBodyPos = targetBallPos + targetAngle/TWO_PI * CentreOfGravityHeight;

		// compute target speed of ball (considering the dynamic tilt angle)
		float targetBodySpeed = (targetBodyPos - lastTargetBodyPos)/dT;

		// compute target speed of ball (considering the dynamic tilt angle)
		float targetBallSpeed = (targetBallPos - lastTargetBallPos)/dT;

		// compute ABSOLUTE position of the body, assume sin(x) = x for small angle
		float absBodyPos = absBallPos + sensorAngle*CentreOfGravityHeight;

		// compute acceleration the robot is supposed to have
		float targetAbsAccel = (targetSpeed - lastTargetSpeed)/dT;

		// compute the ABSOLUTE velocity the robot's body really has
		float absBodySpeed = (absBodyPos - lastAbsBodyPos)/dT;

		// compute the ABSOLUTE velocity the ball has
		float absBallSpeed = (absBallPos - lastAbsBallPos)/dT;

		// compute the ABSOLUTE acceleration the robot's body really has
		// bodyAccel = inputBodyAccel.update((absBodySpeed-lastBodySpeed) / dT);
		absBodyAccel = (absBodySpeed-lastBodySpeed) / dT;

		// compute the ABSOLUTE acceleration the robot's base really has
		// ballAccel = inputBallAccel.update((currentSpeed-lastBallSpeed) / dT);
		absBallAccel = (currentSpeed-lastBallSpeed) / dT;

		if (memory.persistentMem.logConfig.debugBalanceLog) {
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
		}


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

		// compute the delta of the as-is and to-be speed (positive means tobe > as-is)
		errorBodyVelocity = targetBodySpeed - absBodySpeed;

		// compute the difference between the as-is and to-be body acceleration (positive means tobe > as-is)
		errorBodyAccel = (targetAccel - absBodyAccel);

		// compute the difference between the as-is and to-be body acceleration (positive means tobe > as-is)
		errorBallAccel = targetAbsAccel - absBallAccel;

		// compute error against centripedal force, which is f=omega*v*m*c, where m*c is the weight
		float errorCentripedal = targetOmega * targetSpeed;

		// now multiply all deltas in each state variable with the according weight
		float error_tilt			= memory.persistentMem.ctrlConfig.angleWeight 		 * errorAngle;
		float error_angular_speed	= memory.persistentMem.ctrlConfig.angularSpeedWeight * errorAngularVelocity;

		float error_ball_position 	= memory.persistentMem.ctrlConfig.ballPositionWeight * errorBallPosition;
		float error_ball_velocity 	= memory.persistentMem.ctrlConfig.ballVelocityWeight * errorBallVelocity; // [0]
		float error_ball_accel		= memory.persistentMem.ctrlConfig.ballAccelWeight 	 * errorBallAccel;

		float error_body_position	= memory.persistentMem.ctrlConfig.bodyPositionWeight * errorBodyPosition; // [0]
		float error_body_velocity	= memory.persistentMem.ctrlConfig.bodyVelocityWeight * errorBodyVelocity;
		float error_body_accel		= memory.persistentMem.ctrlConfig.bodyAccelWeight    * errorBodyAccel;    // [0]

		float error_centripedal     = memory.persistentMem.ctrlConfig.omegaWeight        * errorCentripedal;

		// sum up all weighted errors
		float error =       -error_tilt - error_angular_speed
							-error_ball_velocity - error_ball_position - error_ball_accel
							-error_body_velocity - error_body_position - error_body_accel
							-error_centripedal;

		if (memory.persistentMem.logConfig.debugBalanceLog) {
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
		accel = constrain(error,-MaxBotAccel, MaxBotAccel);

		// accelerate only if not yet on max speed
		if ((sgn(speed) != sgn(accel)) ||
			(abs(speed) < MaxBotSpeed))
			speed += accel * dT;


		lastTargetAngle = targetAngle;
		lastAbsBodyPos = absBodyPos;
		lastAbsBallPos = absBallPos;
		lastTargetBodyPos = targetBodyPos;
		lastTargetBallPos = targetBallPos;
		lastBodySpeed = absBodySpeed;
		lastBallSpeed = absBallSpeed;
		lastTargetSpeed = targetSpeed;

		// in order to increase gain of state controller, filter with FIR 20Hz 4th order
		filteredSpeed = outputSpeedFilter.update(speed);
		if (memory.persistentMem.logConfig.debugBalanceLog) {
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
	planeX.init();
	planeY.init();
	rampedTargetMovement.init();

}


void StateController::update(float dT, const BotMovement& currentMovement,
							 const IMUSample& sensorSample,
							 const BotMovement& targetBotMovement) {
	// ramp up target speed and omega with a trapezoid profile of constant acceleration
	rampedTargetMovement.rampUp(targetBotMovement, dT);
	if (memory.persistentMem.logConfig.debugBalanceLog)
		logger->print("planeX:");
	planeX.update(dT, currentMovement.speedX, rampedTargetMovement.speedX, rampedTargetMovement.accelX, currentMovement.omega, rampedTargetMovement.omega, sensorSample.plane[Dimension::X].angle, sensorSample.plane[Dimension::X].angularVelocity);

	if (memory.persistentMem.logConfig.debugBalanceLog) {
		logger->println();
		logger->print("planeY:");
	}
	planeY.update(dT, currentMovement.speedY, rampedTargetMovement.speedY, rampedTargetMovement.accelY, currentMovement.omega, rampedTargetMovement.omega, sensorSample.plane[Dimension::Y].angle, sensorSample.plane[Dimension::Y].angularVelocity);
	if (memory.persistentMem.logConfig.debugBalanceLog) {
		logger->println();
	}
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

