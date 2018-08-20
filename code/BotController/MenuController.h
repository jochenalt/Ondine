/*
 * MenuController.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef MENUCONTROLLER_H_
#define MENUCONTROLLER_H_


class Menuable {
public:
	Menuable () {};
	virtual ~Menuable () {};

	virtual void printHelp();
	virtual void menuLoop(char ch) {};
};

class MenuController {
public:
	MenuController() {};
	~MenuController() {};

	void loop();
	void setup();

	void registerMenu(const Menuable* menu);
	void activateMenu(const Menuable* menu);

private:
	static const int MaxNumberOfMenues = 16;
	Menuable* menus[MaxNumberOfMenues];
	int menuSize = 0;
	int activeMenu = -1;
};



#endif /* MENUCONTROLLER_H_ */
