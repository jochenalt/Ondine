/*
 * BotController.h
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#ifndef BOTCONTROLLER_H_
#define BOTCONTROLLER_H_

#include <MenuController.h>
#include <Engine.h>
#include <Kinematics.h>

class BotController : public Menuable {
public:
	void setup();
	void loop();
	BotController() {};
	virtual ~BotController() {};

	void printHelp();
	void menuLoop(char ch);

private:
	Engine engine;
	Kinematix kinematics;
	MenuController menuController;
};

#endif /* BOTCONTROLLER_H_ */
