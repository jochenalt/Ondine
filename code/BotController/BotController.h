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
#include <Engine.h>
#include <IMU.h>
#include <Kinematics.h>
#include <BrushedMotorDriver.h>
#include <Power.h>

class BotController : public Menuable {
public:
	enum Mode { OFF, BALANCE };
	void setup();
	void loop();
	BotController() {};
	virtual ~BotController() {};

	void printHelp();
	void menuLoop(char ch);

	// turn on/off the balancing mode
	void balanceMode(Mode mode) {
		this->mode = mode;
	}
private:
	Engine engine;
	Kinematix kinematics;
	MenuController menuController;
	IMU imu;
	StateController state;
	BotMovement targetBotMovement;
	BrushedMotorDriver lifter;
	Power power;
	Mode mode = OFF;
};

#endif /* BOTCONTROLLER_H_ */
