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
};

class StateControllerConfig {
public:
	void null();
	void initDefaultValues();
	void print() {
		logger->println("state controller configuration:");
		logger->print("   angleWeight:");
		logger->print(angleWeight);
		logger->print(" angularSpeedWeight:");
		logger->println(angularSpeedWeight);
		logger->print("   ballVelocityWeight:");
		logger->print(ballVelocityWeight);
		logger->print(" ballPositionWeight:");
		logger->print(ballPositionWeight);
		logger->print(" ballAccelWeight:");
		logger->println(ballAccelWeight);
		logger->print("   bodyVelocityWeight:");
		logger->print(bodyVelocityWeight);
		logger->print(" bodyPositionWeight:");
		logger->print(bodyPositionWeight);
		logger->print(" bodyAccelWeight:");
		logger->println(bodyAccelWeight);
		logger->print("   omegaWeight:");
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
	PIDControllerConfig pid_setup;

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
