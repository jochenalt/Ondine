/*
 * MenuController.cpp
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <Util.h>
#include <MenuController.h>


void Menuable::registerMenuController(MenuController* newMenuCtrl) {
	menuCtrl = newMenuCtrl;
	menuCtrl->registerMenu(this);
}

void MenuController::setup() {
	menuSize = 0;
	activeMenuStackPtr = 0;
	activeMenuStack[activeMenuStackPtr] = 0;
}

void MenuController::registerMenu(const Menuable* menu) {
	menus[menuSize++] = (Menuable*)menu;
}

void Menuable::deactivateMenu() {
	menuCtrl->deactivateMenu();
}

void Menuable::activateMenu() {
	menuCtrl->activateMenu(this);
}

void MenuController::deactivateMenu() {
	if (activeMenuStackPtr == 0)
		fatalError("MC stack underflow");

	activeMenuStackPtr--;
	menus[activeMenuStack[activeMenuStackPtr]]->printHelp();
}

void MenuController::activateMenu(const Menuable* menu) {
	if (activeMenuStackPtr == MaxNumberOfMenues)
		fatalError("MC stack overflow");

	for (int i = 0;i<menuSize;i++) {
		if (menu == menus[i]) {
			activeMenuStack[++activeMenuStackPtr] = i;
			menus[activeMenuStack[activeMenuStackPtr]]->printHelp();
			break;
		}
	}

	// fatal, reset everything
	activeMenuStackPtr = 0;
	activeMenuStack[activeMenuStackPtr] = 0;

}

void MenuController::loop() {
	if 	(Serial1.available()) {
		char ch = Serial1.read();
		menus[activeMenuStack[activeMenuStackPtr]]->menuLoop(ch);
	}
}
