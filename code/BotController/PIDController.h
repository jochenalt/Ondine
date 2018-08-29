/*
 * PIDController.h
 *
 *  Created on: 29.08.2018
 *      Author: JochenAlt
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <Arduino.h>

class PIDControllerConfig {
public:
	PIDControllerConfig() {
		Kp = 0;
		Ki = 0;
		Kd = 0;
		K_s = 0;
		T_u = 0;
		T_g = 0;
	};

	virtual ~PIDControllerConfig() {};
	PIDControllerConfig (float Kp, float Ki, float Kd) {
		set(Kp,Ki,Kd);
	}

	void set(float Kp, float Ki, float Kd) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	void zieglerAndNicols()  {
		// Ziegler and Nicols, according to https://rn-wissen.de/wiki/index.php/Regelungstechnik
		Kp = (0.9 / K_s) * (T_g / T_u);
		Ki = Kp / (3.3 * T_u);
		Ki = Kp*0.5*T_u;
	}

	float Kp;
	float Ki;
	float Kd;

	// Ziegler and Nicols
	float K_s;
	float T_u;
	float T_g;
};

class PIDController {
public:
	PIDController() {
		integrative = 0;
	}
	virtual ~PIDController() {};
	void reset() {
		integrative = 0;
	}

	float update (PIDControllerConfig& params, float error, float dT, float min, float max) {
		float pOut = params.Kp*error;
		integrative += error * dT;
		integrative = constrain(integrative, min, max);
		float iOut = params.Ki* integrative;
	    float derivative = (error - lastError) / dT;
	    double dOut = params.Kd * derivative;
		lastError = error;
		float out = pOut + iOut + dOut;
		out = constrain(out, min, max);
		return out;
	}
	float integrative = 0;
	float lastError = 0;
};

class DynamicPIDController {
public:
	DynamicPIDController () {};
	virtual ~DynamicPIDController () {};

	float update(float error, float dT, float speedRatio, float min, float max) {
		float positionRatio = 1.0 - speedRatio;
		PIDControllerConfig config (position.Kp*positionRatio + speed.Kp* speedRatio,
									position.Ki*positionRatio + speed.Ki* speedRatio,
									position.Kd*positionRatio + speed.Kd* speedRatio);
		return pid.update(config, error, dT, min, max);
	}

	PIDController pid;

	PIDControllerConfig position;
	PIDControllerConfig speed;
};

#endif /* PIDCONTROLLER_H_ */
