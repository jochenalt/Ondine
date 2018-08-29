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


class StateControllerConfig {
public:
	void null();
	void initDefaultValues();
	void print();

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
	void initDefaultValues() {
		pid.Kp = 5.0;
		pid.Ki = 2.0;
		pid.Kd = 0.0;

		pid.K_s = 1.0;
		pid.T_u = 0.05;
		pid.T_g = 0.1;
	}

	void print();

	// PID values for control at 0 rev/s
	PIDControllerConfig pid;
};

class IMUConfig {
	public:
		void initDefaultValues() {
			offsetRawX = 541;
			offsetRawY = -571;
			offsetRawZ = -397;
		}

	float offsetRawX;
	float offsetRawY;
	float offsetRawZ;
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
		} persistentMem;
};

extern BotMemory memory;

#endif //__BOTMEMORY_H__
