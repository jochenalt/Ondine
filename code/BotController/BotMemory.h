/* 
* BotMemory.h
*
* Persistent Memory of Walter. Stores actuator configuration and logging state in EEPROM.
*
* Author: JochenAlt
*/


#ifndef __BOTMEMORY_H__
#define __BOTMEMORY_H__
#include "Arduino.h"
#include "MemoryBase.h"
#include "PIDController.h"
#include "StateController.h"
#include "BrushlessMotorDriver.h"
#include "IMU.h"

class BotMemory;
extern BotMemory memory;

class LogConfig {
public:
	void null();
	void initDefaultValues();
	void print();
	boolean performanceLog;
	boolean calibrationLog;
	boolean debugBalanceLog;
	boolean debugStateLog;
};



class BotMemory : public MemoryBase {
	public:
		// initialize  default values of memory for the very first start
		BotMemory();
		void println();
		static void setDefaults();

		struct {
			StateControllerConfig ctrlConfig;
			MotorConfig motorControllerConfig;
			IMUConfig imuControllerConfig;
			LogConfig logConfig;
		} persistentMem;
};

extern BotMemory memory;

#endif //__BOTMEMORY_H__
