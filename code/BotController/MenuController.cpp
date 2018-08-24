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

void Menuable::popMenu() {
	menuCtrl->popMenu();
}

void Menuable::pushMenu() {
	menuCtrl->pushMenu(this);
}

void MenuController::popMenu() {
	if (activeMenuStackPtr == 0)
		fatalError("MC stack underflow");
	else {
		activeMenuStackPtr--;
		menus[activeMenuStack[activeMenuStackPtr]]->printHelp();
	}
}

void MenuController::pushMenu(const Menuable* menu) {
	if (activeMenuStackPtr == MaxNumberOfMenues)
		fatalError("MC stack overflow");
	else {
		for (int i = 0;i<menuSize;i++) {
			if (menu == menus[i]) {
				activeMenuStackPtr++;
				activeMenuStack[activeMenuStackPtr] = i;
				menus[activeMenuStack[activeMenuStackPtr]]->printHelp();
				return;
			}
		}

		fatalError("menu not found");
		// fatal, reset everything
		activeMenuStackPtr = 0;
		activeMenuStack[activeMenuStackPtr] = 0;
	}
}

void MenuController::loop() {
	if 	(command->available()) {
		char ch = command->read();
		if (ch == 27)
			popMenu();
		else
			menus[activeMenuStack[activeMenuStackPtr]]->menuLoop(ch);
	}
}
