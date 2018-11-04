/*
 * BotController.h
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#ifndef BOTCONTROLLER_H_
#define BOTCONTROLLER_H_

#include <types.h>
#include <MenuController.h>
#include <StateController.h>
#include <BallDrive.h>
#include <IMU.h>
#include <Kinematics.h>
#include <BrushedMotorDriver.h>
#include <PowerRelay.h>
#include <TimePassedBy.h>

class BotController : public Menuable {
public:
	enum Mode { OFF, BALANCE };
	void setup();
	void loop();
	static BotController& getInstance() {
		static BotController instance;
		return instance;
	}
	virtual ~BotController() {};

	void printHelp();
	void menuLoop(char ch, bool continously);

	void powerEngine(bool doIt);
	bool isEnginePowered();

	// turn on/off the balancing mode
	void balanceMode(Mode mode) {
		this->mode = mode;

		// set current position as starting psition
		ballDrive.reset();
		state.reset();
	}

	bool isBalancing() {
		return mode == BALANCE;
	}
private:
	BotController() {};

	BallDrive ballDrive;
	MenuController menuController;
	IMU imu;
	StateController state;
	BotMovement targetBotMovement;
	BrushedMotorDriver lifter;
	TimePassedBy performanceLogTimer;
	Mode mode = OFF;
	TimePassedBy logTimer;
};

#endif /* BOTCONTROLLER_H_ */
