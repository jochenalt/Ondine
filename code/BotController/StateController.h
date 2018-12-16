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
	float intAngleWeight;
	float ballPosIntegratedWeight;
	float ballPositionWeight;
	float ballVelocityWeight;
	float ballAccelWeight;

	float omegaWeight;
};



class ControlPlane {
	public:
		ControlPlane() {};

		ControlPlane(Dimension dim) {
			this->dim = dim;
		}
		void reset ();
		float lastTargetAngle = 0;
		float lastAngle = 0;
		float lastTargetBodyPos = 0;
		float lastTargetBallPos = 0;
		float lastBallPos = 0;
		float lastBallSpeed = 0;
		float lastBodyPos = 0;
		float lastBodySpeed = 0;
		float lastBodyAccel = 0;
		float error = 0;
		float accel =0;
		float speed = 0;			// speed in x direction [m/s]
		float filteredSpeed = 0;
		float posErrorIntegrated = 0;
		float tiltErrorIntegrated = 0;

		float totalTiltError = 0;
		float totalPositionError = 0;
		FIR::Filter  posFilter;
		FIR::Filter outputSpeedFilter;
		LowPassFilterFrequency outputSpeedFilter2;
		Dimension dim;

		// compute new speed in the given pane, i.e. returns the error correction that keeps the bot balanced and on track
		void update(bool log,float dT,
					const State& current, const State& target,
						float pActualOmega, float pToBeOmega,
					const IMUSamplePlane &sensor);
		void print();
		float getBodyPos();
		float getBallPos();
		float getAccel();
		float getPosError();
		float getTiltError();

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

	float getTiltErrorX() {
		return planeX.getTiltError();
	}
	float getTiltErrorY() {
		return planeY.getTiltError();
	}

	float getPosErrorX() {
		return planeX.getPosError();
	}
	float getPosErrorY() {
		return planeY.getPosError();
	}

	float getPosX() {
		return planeX.getBallPos();
	}
	float getPosY() {
		return planeY.getBallPos();
	}

	float getAccelX() {
		return planeX.getAccel();
	}

	float getAccelY() {
		return planeY.getAccel();
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
