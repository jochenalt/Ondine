#ifndef _COMMON_H
#define _COMMON_H


// all definitions that are used by bot contoller and by webserver


// i2c address of bot controllers
const int BotControllerI2CAddress = 0x17;

// all register addresses used as commands from webserver to bot controller
const static int BotCtrlCmd_SerialCommand = 0;	// consider the communicated string as menu input equivalent to serial input

#endif
