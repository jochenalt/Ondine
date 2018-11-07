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
#include <TimePassedBy.h>

class ControlPlane {
	public:
		void reset ();
		float lastTargetAngle;
		float lastTargetBodyPos;
		float lastTargetBallPos;
		float lastTargetBallSpeed;
		float lastBallPos;
		float lastBodyPos;		// absolute as-is position of last loop
		float lastBodySpeed;
		float lastBallSpeed;
		float lastTargetBodySpeed;

		float speed;			// speed in x direction [mm/s]
		float error;			// current error of control loop used to compute the acceleration
		float accel;			// final acceleration out of the control loop
		float filteredSpeed;

		FIR::Filter outputSpeedFilter;
		FIR::Filter inputBallAccel;
		FIR::Filter inputBodyAccel;

		// compute new speed in the given pane, i.e. returns the error correction that keeps the bot balanced and on track
		void update(float dT,
					const State& current, const State& target,
						float pActualOmega, float pToBeOmega,
					const IMUSamplePlane &sensor);
		void print();
		float getBodyPos();
		float getBallPos();

		TimePassedBy logTimer;
};


class StateController : public Menuable {
public:
	StateController() {};
	virtual ~StateController() {};

	void setup(MenuController* menuCtrl);
	void loop();

	void reset();
	virtual void printHelp();
	virtual void menuLoop(char ch, bool continously);

	void update( float dT, const BotMovement& currentMovement,
			 	 const IMUSample& sensorSample,
				 const BotMovement& targetMovement);

	float getSpeedX();
	float getSpeedY();
	float getOmega();

	float getPosX() {
		return planeX.getBallPos();
	}
	float getPosY() {
		return planeY.getBallPos();
	}

	// return average time consumed by update in [s]
	float getAvrLoopTime() { return avrLoopTime; };

private:
	ControlPlane planeX;
	ControlPlane planeY;

	BotMovement rampedTargetMovement;
	float avrLoopTime = 0;
};

#endif /* STATECONTROLLER_H_ */
