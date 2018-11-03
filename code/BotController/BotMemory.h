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
};

class StateControllerConfig {
public:
	void null();
	void initDefaultValues();
	void print() {
		logger->println("state controller configuration:");
		logger->print("   angle=");
		logger->print(angleWeight);
		logger->print(" angularSpeed=");
		logger->println(angularSpeedWeight);
		logger->print("   ballVelocity=");
		logger->print(ballVelocityWeight);
		logger->print(" ballPosition=");
		logger->print(ballPositionWeight);
		logger->print(" ballAccel=");
		logger->println(ballAccelWeight);
		logger->print("   bodyVelocity=");
		logger->print(bodyVelocityWeight);
		logger->print(" bodyPosition=");
		logger->print(bodyPositionWeight);
		logger->print(" bodyAccel=");
		logger->println(bodyAccelWeight);
		logger->print("   omega=");
		logger->println(omegaWeight);
	}

	float angleWeight;
	float angularSpeedWeight;
	float ballVelocityWeight;
	float ballPositionWeight;
	float ballAccelWeight;
	float bodyVelocityWeight;
	float bodyPositionWeight;
	float bodyAccelWeight;
	float omegaWeight;
};


class MotorConfig {
public:
	void initDefaultValues();
	void print();

	// PID values for control at 0 rev/s
	PIDControllerConfig pid_position;
	PIDControllerConfig pid_speed;
	PIDControllerConfig pid_lifter;
};

class IMUConfig {
	public:
		void initDefaultValues();

		void print();

	float nullOffsetX;
	float nullOffsetY;
	float nullOffsetZ;
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
