#include "MemoryBase.h"
#include "BotMemory.h"
#include <EEPROM.h>

BotMemory memory;

BotMemory::BotMemory()
: MemoryBase((void*)&(persistentMem),sizeof(BotMemory::persistentMem)) {
	// initialization for the very first start, when EEPROM is not yet initialized
	BotMemory::setDefaults();
}

void BotMemory::setDefaults() {
	memory.persistentMem.ctrlConfig.initDefaultValues();
	memory.persistentMem.motorControllerConfig.initDefaultValues();
	memory.persistentMem.imuControllerConfig.initDefaultValues();
}


void BotMemory::println() {
}


void StateControllerConfig::initDefaultValues() {
	angleWeight				= 39.0;
	angularSpeedWeight		= 21.00;
	ballPositionWeight			= 1.5;
	ballVelocityWeight			= 0.0;
	ballAccelWeight				= 1.3;
	bodyPositionWeight		= 0.0;
	bodyVelocityWeight		= 9.0;
	bodyAccelWeight			= 0.0;
	omegaWeight				= 0.0;
}

