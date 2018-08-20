/*
 * MenuController.cpp
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"
#include <MenuController.h>

void MenuController::setup() {
	menuSize = 0;
	activeMenu = -1;
}

void MenuController::registerMenu(const Menuable* menu) {
	menus[menuSize++] = (Menuable*)menu;
}

void MenuController::activateMenu(const Menuable* menu) {
	for (int i = 0;i<activeMenu;i++) {
		if (menu == menus[i]) {
			menuSize = i;
			break;
		}
	}
	activeMenu = 0;
}


void MenuController::loop() {
	if 	(Serial1.available()) {
		char ch = Serial1.read();
		if (activeMenu >= 0)
			menus[activeMenu]->menuLoop(ch);
	}
}
