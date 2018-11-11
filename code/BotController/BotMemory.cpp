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
	memory.persistentMem.logConfig.initDefaultValues();
}


void BotMemory::println() {
	logger->print("EEPROM memory (V");
	logger->print(EEPROMVersion());
	logger->println("):");

	persistentMem.ctrlConfig.print();
	logger->println();
	persistentMem.motorControllerConfig.print();;
	logger->println();
	persistentMem.imuControllerConfig.print();
	logger->println();
	persistentMem.logConfig.print();
	logger->println();
}


void LogConfig::null() {
	performanceLog = false;
	calibrationLog = false;
	debugBalanceLog = false;
	debugStateLog = false;
}

void LogConfig::initDefaultValues() {
	performanceLog = true;
	calibrationLog = false;
	debugBalanceLog = true;
	debugStateLog = false;
}

void LogConfig::print() {
	logger->println("logging:");
	logger->print("   perf   :");
	logger->println(performanceLog?"true":"false");
	logger->print("   calib  :");
	logger->println(calibrationLog?"true":"false");
	logger->print("   balance:");
	logger->println(debugBalanceLog?"true":"false");
	logger->print("   state  :");
	logger->println(debugStateLog?"true":"false");

}
