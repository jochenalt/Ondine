/*
 * PIDController.h
 *
 *  Created on: 29.08.2018
 *      Author: JochenAlt
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <Arduino.h>
#include <Util.h>

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
		Kd = Kp*0.5*T_u;
	}

	float Kp;
	float Ki;
	float Kd;

	// Ziegler and Nicols (impulseanswer)
	float K_s; // amplification = dSpeed / dT = acceleration
	float T_u; // dead time, delay
	float T_g; // execution time
};

/* self-tuning PID Controller with fuzzy controller
 *
 * according to "Brushless DC motor tracking control using
 * self-tuning fuzzy PID control and model reference adaptive control"
 * https://core.ac.uk/download/pdf/82512668.pdf
 */
class PIDController {
public:

	PIDController() {
		integrativeError = 0;
	}
	virtual ~PIDController() {};

	float update (PIDControllerConfig& params, float error, float dT, float min, float max);

	void reset();

	float integrativeError = 0;
	float lastError = 0;
};

/* PID controller that consist of two sets of configuration params.
 * One is used for position control, the other for speed control
 * When pid is running, current speed is used to interpolate the PID params sets
 * By this, a high gain with low speeds and low gain at high speeds can be implemented
 */
class SpeedGainPIDController : public PIDController {
public:
	SpeedGainPIDController () {};
	virtual ~SpeedGainPIDController () {};

	float update(const PIDControllerConfig &position, const PIDControllerConfig &speed, float min, float max, float speedRatio,
				float error, float dT) {
		speedRatio *= speedRatio;
		float positionRatio = 1.0 - speedRatio;
		PIDControllerConfig config (position.Kp*positionRatio + speed.Kp* speedRatio,
									position.Ki*positionRatio + speed.Ki* speedRatio,
									position.Kd*positionRatio + speed.Kd* speedRatio);
		return PIDController::update(config, error, dT, min, max);
	}
	void reset() {
		PIDController::reset();
	}
};



#endif /* PIDCONTROLLER_H_ */
