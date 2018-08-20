/*
 * Engine.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include <MenuController.h>
#include <BLDCController.h>

class Engine : public Menuable  {
public:
	Engine() {};
	virtual ~Engine() {};

	void loop();

	virtual void setup(MenuController& menuCtrl);
	virtual void menuLoop(char ch);
	virtual void printHelp();
private:
	BLDCController* ctrl[3] = { NULL, NULL, NULL };

	int activeMenuWheel = 0;
	MenuController* menuCtrl = NULL;
};

#endif /* ENGINE_H_ */
