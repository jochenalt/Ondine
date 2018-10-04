#ifndef UTIL_H_
#define UTIL_H_

#include <Arduino.h>
#include <Wire.h>
#include <LogStream.h>

// i2c address of bot controllers
const int ctrlCommAddress = 0x17;

// communication line to bot controller
extern TwoWire* ctrlComm;
extern HardwareSerial* botControllerLogs;

extern LogStream* logger;
const static int CmdSerialCommand = 0;
void sendCommandAsync(int adr, String cmd);
String requestResponse(int numberOfBytes);
void communicate();

#endif
