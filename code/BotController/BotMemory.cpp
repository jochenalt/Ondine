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
	logger->println("EEPROM memory:");
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
	logger->println("logging:");
	logger->print("   perf   :");
	logger->println(performanceLog?"true":"false");
	logger->print("   calib  :");
	logger->println(calibrationLog?"true":"false");

}

void MotorConfig::initDefaultValues() {
	// at slow speeds PID controller is aggressivly keeping the position
	pid_position.Kp = 3.0;
	pid_position.Ki = 1.2;
	pid_position.Kd = 0.000;

	pid_speed.Kp = .8;
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
	nullOffsetY = -0;
	nullOffsetZ = -0;
}

void IMUConfig::print() {
	logger->println("imu configuration");
	logger->print("   null=(");
	logger->print(nullOffsetY);
	logger->print(",");
	logger->print(nullOffsetY);
	logger->print(",");
	logger->print(nullOffsetZ);
	logger->print("))");
}
