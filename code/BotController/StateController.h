/*
 * StateController.h
 *
 *  Created on: 23.08.2018
 *      Author: JochenAlt
 */

#ifndef STATECONTROLLER_H_
#define STATECONTROLLER_H_

#include <Filter/FIRFilter.h>
#include <Filter/IIRFilter.h>
#include <Filter/ComplementaryFilter.h>

#include <types.h>
#include <setup.h>
#include <IMU.h>
#include <TimePassedBy.h>


class StateControllerConfig {
public:
	void null();
	void initDefaultValues();
	void print();
	float angleWeight;
	float angularSpeedWeight;

	float ballPosIntegratedWeight;
	float ballPositionWeight;
	float ballVelocityWeight;

	float omegaWeight;
};



class ControlPlane {
	public:
		void reset ();
		float lastTargetAngle;
		float lastAngle;
		float lastTargetBodyPos;
		float lastTargetBallPos;
		float lastBallPos;
		float lastBallSpeed;

		float speed;			// speed in x direction [m/s]
		float filteredSpeed;
		float posErrorIntegrated;

		FIR::Filter outputSpeedFilter;
		LowPassFilter1stOrder outputSpeedFilter2;

		// compute new speed in the given pane, i.e. returns the error correction that keeps the bot balanced and on track
		void update(bool log,float dT,
					const State& current, const State& target,
						float pActualOmega, float pToBeOmega,
					const IMUSamplePlane &sensor);
		void print();
		float getBodyPos();
		float getBallPos();
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

	void update( 	float dT,
					const IMUSample& sensorSample,
					const BotMovement& currentMovement,
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
	TimePassedBy logTimer;

};

#endif /* STATECONTROLLER_H_ */
