/*
 * BotController.h
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#ifndef BOTCONTROLLER_H_
#define BOTCONTROLLER_H_

#include <types.h>
#include <libraries/MenuController.h>
#include <StateController.h>
#include <BallDrive.h>
#include <IMU.h>
#include <Kinematics.h>
#include <BrushedMotorDriver.h>
#include <PowerRelay.h>
#include <TimePassedBy.h>

class BotController : public Menuable {
public:
	enum BotMode { OFF, BALANCING };

	// singleton
	static BotController& getInstance() {
		static BotController instance;
		return instance;
	}

	// to be called before anything else happens.
	// initializes motors, imu, state controller, everything
	void setup();

	// to be called in main loop as often as possible.
	// takes are of fetching the IMU values and doing the balancing part
	void loop();

	virtual ~BotController() {};

	void printHelp();
	void menuLoop(char ch, bool continously);

	// turn the engine's power  on/off
	void powerEngine(bool doIt);
	bool isEnginePowered();

	// turn on/off the balancing mode
	void balanceMode(BotMode mode) {
		if (mode == this->mode)
			return;

		if (this->mode == OFF) {
			if (!imu.isValid()) {
				fatalError("IMU is not working properly");
				return;
			}
			if (!ballDrive.isPowered()) {
				fatalError("power is not turned on");
				return;
			}
			if (!ballDrive.isEnabled()) {
				fatalError("engine is not engaged");
				return;
			}
		}

		this->mode = mode;

		// set current position as starting psition
		ballDrive.reset();
		state.reset();
		currentMovement.reset();
	}

	void setTarget(const BotMovement& target);

	bool isBalancing() {
		return mode == BALANCING;
	}
	StateController& getStateController() { return state; };
private:
	BotController() {};

	BallDrive ballDrive;
	MenuController menuController;
	IMU imu;
	StateController state;
	BotMovement currentMovement;
	BotMovement targetBotMovement;
	BrushedMotorDriver lifter;
	TimePassedBy performanceLogTimer;
	BotMode mode = OFF;
	TimePassedBy logTimer;
	float avrLoopTime = 0;
};

#endif /* BOTCONTROLLER_H_ */
