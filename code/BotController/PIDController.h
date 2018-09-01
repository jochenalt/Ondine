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
		Ki = Kp*0.5*T_u;
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

	void turnFuzzyAdaption(boolean doit, float dT) {
		fuzzyOn = doit;
		fuzzydT = dT;
	}

	bool isFuzzy() {
		return fuzzyOn;
	}

	float update (PIDControllerConfig& params, float error, float dT, float min, float max);

	void reset();

	float integrativeError = 0;
	float lastError = 0;

private:
	enum LinguisticLabel {NegativeBig,  NegativeSmall,  NoInput, PositiveSmall, PositiveBig};
	enum OutputLabel {Zero, MediumSmall,Small, Medium, Big, MediumBig, VeryBig};

	const char* getLinguisticLabelName(LinguisticLabel i) {
		switch (i) {
		case NegativeBig: return "NB";
		case NegativeSmall: return "NS";
		case NoInput: return "Z";
		case PositiveSmall: return "PS";
		case PositiveBig: return "PB";
		}
		return NULL;
	}
	const char* getOutputLabelName(OutputLabel i) {
		switch (i) {
		case Zero: return "Z";
		case MediumSmall: return "MS";
		case Small: return "S";
		case Medium: return "M";
		case Big: return "B";
		case MediumBig: return "MB";
		case VeryBig: return "VB";
		}
		return NULL;
	}

	typedef  OutputLabel RuleBaseType[5][5];
	// rule based sets coming from Self-tuningFuzzyPIDController
	const RuleBaseType ruleBaseKp = {
			{ VeryBig, VeryBig,      VeryBig,    VeryBig, VeryBig },
			{  	  Big,     Big,          Big,  MediumBig, VeryBig },
			{  	  Zero,   Zero,   MediumSmall,     Small,   Small },
			{  	  Big,   Big,             Big, MediumBig,  VeryBig},
			{ VeryBig, VeryBig,      VeryBig,    VeryBig, VeryBig }};
	const RuleBaseType ruleBaseKi = {
			{     Medium,       Medium,   Medium,      Medium,      Medium},
			{       Small,       Small,    Small,       Small,       Small },
			{ MediumSmall, MediumSmall,     Zero, MediumSmall, MediumSmall },
			{  	    Small,       Small,    Small,       Small,       Small },
			{      Medium,      Medium,   Medium,      Medium,     Medium}};
	const RuleBaseType ruleBaseKd = {
			{        Zero,      Small,     Medium,  MediumBig, VeryBig},
			{       Small,        Big,  MediumBig,  VeryBig,   VeryBig },
			{      Medium,  MediumBig,  MediumBig,  VeryBig,   VeryBig },
			{  	      Big,    VeryBig,    VeryBig,  VeryBig,   VeryBig },
			{     VeryBig,    VeryBig,    VeryBig,  VeryBig,   VeryBig}};

	const float membershipFunction[7][3] =
	                                {{0,	0,	0.5},
	                                 {0.25,0.5, 0.75},
									 {0.5,0.75, 1.0 },
									 {0.75,1.0, 1.25 },
									 {1.0,1.25, 1.5 },
									 {1.25,1.5, 1.65 },
									 {1.5, 2.0, 2.0 }};

	// returns the interpolated value of a triangle function:
	// f(x<=left) = 0, f(middle) = 1, f(x>=right)=0
	float triangleFunction (float input, float left, float middle, float right);
	float centroidDefuzzification(float fuzzyNumberError[5], float fuzzyNumberdError[5], const RuleBaseType &ruleBase);
	void updateFuzzy(PIDControllerConfig config, float error, float dError);

	bool fuzzyOn = false;
	float fuzzydT = 0.1;
	float sumdT = 0;
	float sumError;
	float sumdError;
	int sumLoops  = 0;
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
