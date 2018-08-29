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
		pid_position.Kp = 5.0;
		pid_position.Ki = 2.0;
		pid_position.Kd = 0.0;

		pid_position.K_s = 5.0;
		pid_position.T_u = 0.01;
		pid_position.T_g = 0.1;

		pid_speed.Kp = 0.8;
		pid_speed.Ki = 0.2;
		pid_speed.Kd = 0.0;

		pid_speed.K_s = 1.0;
		pid_speed.T_u = 0.05;
		pid_speed.T_g = 0.1;

	}

	void print();

	// PID values for control at 0 rev/s
	PIDControllerConfig pid_position;
	PIDControllerConfig pid_speed;
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
