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


void StateControllerConfig::initDefaultValues() {

	// initialize the weights used for the state controller per
	// computed state dimension
	// state controller consists of
	// (angle, angular speed,
	//  ball position, ball speed, ball acceleration,
	//  body position, body speed, body acceleration,
	// omega)
	angleWeight				= 0*39.0 + 10;
	angularSpeedWeight		= 0*21.00;

	ballPositionWeight		= 0*1.5;
	ballVelocityWeight		= 0.0;
	ballAccelWeight			= 0*1.3;

	bodyPositionWeight		= 0.0;
	bodyVelocityWeight		= 0*9.0;
	bodyAccelWeight			= 0.0;

	omegaWeight				= 0.0;
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

void MotorConfig::initDefaultValues() {
	// at slow speeds PID controller is aggressivly keeping the position
	pid_position.Kp = 1.5;
	pid_position.Ki = 1.2;
	pid_position.Kd = 0.0;

	pid_speed.Kp = .9;
	pid_speed.Ki = 0.5;
	pid_speed.Kd = 0.02;

	pid_lifter.Kp = 0.01;
	pid_lifter.Ki = 0.005;
	pid_lifter.Kd = 0.0;

}

void MotorConfig::print() {
	logger->println("motor controller configuration:");
	logger->print("   PID (speed=0)  : ");
	logger->print("P=");
	logger->print(pid_position.Kp);
	logger->print(" I=");
	logger->print(pid_position.Ki);
	logger->print(" D=");
	logger->println(pid_position.Kd);
	logger->print("   PID (speed=max): ");
	logger->print("P=");
	logger->print(pid_speed.Kp);
	logger->print(" I=");
	logger->print(pid_speed.Ki);
	logger->print(" D=");
	logger->println(pid_speed.Kd);
	logger->println();
	logger->println("lifter controller configuration:");
	logger->print("   PID (speed=max): ");
	logger->print("P=");
	logger->print(pid_lifter.Kp);
	logger->print(" I=");
	logger->print(pid_lifter.Ki);
	logger->print(" D=");
	logger->println(pid_lifter.Kd);

}

void IMUConfig::initDefaultValues() {
	nullOffsetX = 0;
	nullOffsetY = 0;
	nullOffsetZ = 0;
}

void IMUConfig::print() {
	logger->println("imu configuration");
	logger->print("   null=(");
	logger->print(degrees(nullOffsetX));
	logger->print(",");
	logger->print(degrees(nullOffsetY));
	logger->print(",");
	logger->print(degrees(nullOffsetZ));
	logger->print("))");
}
