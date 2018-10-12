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
	persistentMem.ctrlConfig.print();
	logger->println();
	persistentMem.motorControllerConfig.print();;
	logger->println();
	persistentMem.imuControllerConfig.print();
	logger->println();
	persistentMem.logConfig.print();
	logger->println();
}


void StateControllerConfig::initDefaultValues() {

	// initialize the weights used for the state controller per
	// computed state dimension
	// state controller consists of
	// (angle, angular speed,
	//  ball position, ball speed, ball acceleration,
	//  body position, body speed, body acceleration,
	// omega)
	angleWeight				= 39.0;
	angularSpeedWeight		= 21.00;

	ballPositionWeight		= 1.5;
	ballVelocityWeight		= 0.0;
	ballAccelWeight			= 1.3;

	bodyPositionWeight		= 0.0;
	bodyVelocityWeight		= 9.0;
	bodyAccelWeight			= 0.0;

	omegaWeight				= 0.0;
}

void LogConfig::null() {
	performanceLog = false;
	calibrationLog = false;
}

void LogConfig::initDefaultValues() {
	performanceLog = false;
	calibrationLog = false;
}

void LogConfig::print() {
	logger->println("log:");
	logger->print("   perf:");
	logger->println(performanceLog?"true":"false");
	logger->print("   calib:");
	logger->println(calibrationLog?"true":"false");

}
