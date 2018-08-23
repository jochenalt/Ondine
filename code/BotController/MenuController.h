/*
 * MenuController.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef MENUCONTROLLER_H_
#define MENUCONTROLLER_H_


class MenuController;

class Menuable {
	friend class MenuController;
public:
	Menuable () {};
	virtual ~Menuable () {};

	virtual void activateMenu();
	virtual void deactivateMenu();
	virtual void registerMenuController(MenuController* menuCtrl);
	virtual void printHelp();
	virtual void menuLoop(char ch) {};

protected:
	MenuController* menuCtrl = 0;
};

class MenuController {
	friend class Menuable;
public:
	MenuController() {};
	~MenuController() {};

	void loop();
	void setup();


	void registerMenu(const Menuable* menu);
	void activateMenu(const Menuable* menu);
	void deactivateMenu();
private:
	static const int MaxNumberOfMenues = 8;
	Menuable* menus[MaxNumberOfMenues];
	int menuSize = 0;
	int activeMenuStackPtr = 0;
	int activeMenuStack[MaxNumberOfMenues];
};



#endif /* MENUCONTROLLER_H_ */
