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
	boolean debugStateLog;

};

class StateControllerConfig {
public:
	void null();
	void initDefaultValues();
	void print() {
		StateControllerConfig defValue;
		defValue.initDefaultValues();
		logger->println("state controller configuration:");
		logger->print("   angle=");
		logger->print(angleWeight);
		logger->print("(");
		logger->print(defValue.angleWeight);
		logger->print(")");
		logger->print(" angularSpeed=");
		logger->print(angularSpeedWeight);
		logger->print("(");
		logger->print(defValue.angularSpeedWeight);
		logger->println(")");
		logger->print("   ballPos=");
		logger->print(ballPositionWeight);
		logger->print("(");
		logger->print(defValue.ballPositionWeight);
		logger->print(")");
		logger->print(" ballSpeed=");
		logger->print(ballVelocityWeight);
		logger->print("(");
		logger->print(defValue.ballVelocityWeight);
		logger->print(")");
		logger->print(" ballAccel=");
		logger->print(ballAccelWeight);
		logger->print("(");
		logger->print(defValue.ballAccelWeight);
		logger->println(")");
		logger->print("   bodyPosition=");
		logger->print(bodyPositionWeight);
		logger->print("(");
		logger->print(defValue.bodyPositionWeight);
		logger->print(")");
		logger->print(" bodySpeed=");
		logger->print(bodyVelocityWeight);
		logger->print("(");
		logger->print(defValue.bodyVelocityWeight);
		logger->print(")");
		logger->print(" bodyAccel=");
		logger->print(bodyAccelWeight);
		logger->print("(");
		logger->print(defValue.bodyAccelWeight);
		logger->println(")");
		logger->print("   omega=");
		logger->print(omegaWeight);
		logger->print("(");
		logger->print(defValue.omegaWeight);
		logger->println(")");
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
