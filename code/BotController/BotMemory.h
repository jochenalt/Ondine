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
		logger->print("stateConfig(");
		logger->print("angleWeight:");
		logger->print(angleWeight);
		logger->print(" angularSpeedWeight:");
		logger->print(angularSpeedWeight);
		logger->print(" ballVelocityWeight:");
		logger->print(ballVelocityWeight);
		logger->print(" ballPositionWeight:");
		logger->print(ballPositionWeight);
		logger->print(" ballAccelWeight:");
		logger->print(ballAccelWeight);
		logger->print(" bodyVelocityWeight:");
		logger->print(bodyVelocityWeight);
		logger->print(" bodyPositionWeight:");
		logger->print(bodyPositionWeight);
		logger->print(" bodyAccelWeight:");
		logger->print(bodyAccelWeight);
		logger->print(" omegaWeight:");
		logger->print(omegaWeight);
		logger->print(")");

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
	void initDefaultValues() {
		// PID controller at slow speeds is aggressive to keep position
		pid_position.Kp = 2.7;
		pid_position.Ki = 0.5;
		pid_position.Kd = 0.000;

		pid_speed.Kp = .5;
		pid_speed.Ki = 0.5;
		pid_speed.Kd = 0.02;
	}

	void print() {
		logger->print("motorConfig(");
		logger->print("pidPosition(");
		logger->print(pid_position.Kp);
		logger->print(",");
		logger->print(pid_position.Ki);
		logger->print(",");
		logger->print(pid_position.Kd);
		logger->print(") pid_speed(");
		logger->print(pid_speed.Kp);
		logger->print(",");
		logger->print(pid_speed.Ki);
		logger->print(",");
		logger->print(pid_speed.Kd);
		logger->print("))");
	}

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

		void print() {
			logger->print("imu(offset=(");
			logger->print(offsetRawX);
			logger->print(",");
			logger->print(offsetRawY);
			logger->print(",");
			logger->print(offsetRawZ);
			logger->print("))");
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
			LogConfig logConfig;
		} persistentMem;
};

extern BotMemory memory;

#endif //__BOTMEMORY_H__
