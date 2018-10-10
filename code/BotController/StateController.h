/*
 * StateController.h
 *
 *  Created on: 23.08.2018
 *      Author: JochenAlt
 */

#ifndef STATECONTROLLER_H_
#define STATECONTROLLER_H_

#include <Filter/FIRFilter.h>
#include <types.h>
#include <setup.h>
#include <IMU.h>

class ControlPlane {
	public:
		void init () {
			absBallPos = 0;		// absolute position of the base (origin = position when the bot has been switched on)
			lastTargetAngle = 0;
			lastAbsBodyPos = 0; // absolute body position of last loop
			lastAbsBallPos = 0;
			lastBodySpeed = 0;
			lastBallSpeed = 0;
			lastTargetBodyPos = 0;
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

			speedFilter.init(FIR::LOWPASS,
					         0.01 /* allowed ripple in amplitude  */,
							 0.001 /* supression required*/,
							 SampleFrequency,
							 20 /* cut off frequency */);
		}

		float targetAngle;			// expected angle out of acceleration
		float bodyVelocity;			// absolute velocity of body
		float targetBallPos;		// absolute to-be position of the bot
		float absBallPos;			// absolute as-is position of the bot
		float lastTargetAngle;
		float lastTargetBodyPos;
		float lastAbsBallPos;
		float lastAbsBodyPos;		// absolute as-is position of last loop
		float lastBodySpeed;
		float lastBallSpeed;
		float lastTargetSpeed;
		float bodyAccel;
		float ballAccel;
		float errorAngle;
		float errorAngularVelocity;
		float errorBallPosition;
		float errorBodyPosition;
		float errorBallVelocity;
		float errorBodyVelocity;
		float errorBodyAccel;
		float errorBallAccel;

		float speed;			// speed in x direction [mm/s]
		float error;			// current error of control loop used to compute the acceleration
		float accel;			// final acceleration out of the control loop
		float filteredSpeed;

		FIR::Filter speedFilter;

		// compute new speed in the given pane, i.e. returns the error correction that keeps the bot balanced and on track
		void update(float dT,
						float pActualSpeed, float pToBeSpeed, float targetAccel,
						float pActualOmega, float pToBeOmega,
						float pTilt, float pAngularSpeed);
		void print();
};


class StateController : public Menuable {
public:
	StateController() {};
	virtual ~StateController() {};

	void setup(MenuController* menuCtrl);
	void loop();

	virtual void printHelp();
	virtual void menuLoop(char ch);

	void update( float dT, const BotMovement& currentMovement,
			 	 const IMUSample& sensorSample,
				 const BotMovement& targetMovement);

	float getSpeedX();
	float getSpeedY();
	float getOmega();

private:
	ControlPlane planeX;
	ControlPlane planeY;

	BotMovement rampedTargetMovement;
};

#endif /* STATECONTROLLER_H_ */
