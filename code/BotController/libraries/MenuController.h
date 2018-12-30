/*
 * MenuController.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef MENUCONTROLLER_H_
#define MENUCONTROLLER_H_


#include <Arduino.h>
class MenuController;

class Menuable {
	friend class MenuController;
public:
	Menuable () {};
	virtual ~Menuable () {};

	virtual void pushMenu();
	virtual void popMenu();
	virtual void registerMenuController(MenuController* menuCtrl);
	virtual void printHelp();
	virtual void menuLoop(char ch, bool continously)=0;

protected:
	MenuController* menuCtrl = 0;

};

class MenuController {
	friend class Menuable;
public:
	MenuController() {};
	virtual ~MenuController() {};

	void loop();
	void setup();
	void registerMenu(const Menuable* menu);
	void pushMenu(const Menuable* menu);
	void popMenu();
	virtual void menuLoop(char ch,  bool continously) {};
	int getMenuSpeed() { return menuSpeed; };
private:
	static const int MaxNumberOfMenues = 10;
	Menuable* menus[MaxNumberOfMenues];
	int menuSize = 0;
	int activeMenuStackPtr = 0;
	int activeMenuStack[MaxNumberOfMenues];
	int menuSpeed = 0;
	uint32_t lastKeyPressed = 0;
	int continousKeyPressCounter = 0;
};



#endif /* MENUCONTROLLER_H_ */
