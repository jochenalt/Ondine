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
	void reset() {
		integrativeError = 0;
	}

	void turnFuzzyAdaption(boolean doit, float dT) {
		fuzzyOn = doit;
		fuzzydT = dT;
	}
	float update (PIDControllerConfig& params, float error, float dT, float min, float max) {
		float pOut = params.Kp*error;
		integrativeError += error * dT;
		integrativeError = constrain(integrativeError, min, max);
		float iOut = params.Ki* integrativeError;
		float dError = error - lastError;
	    float derivative = dError / dT;
	    double dOut = params.Kd * derivative;
		lastError = error;
		float out = pOut + iOut + dOut;
		out = constrain(out, min, max);

		sumdT += dT;
		sumError += error;
		sumdError += dError;
		sumLoops++;
		if (fuzzyOn && (sumdT > fuzzydT)) {
			updateFuzzy(params, sumError/sumLoops/abs(max), sumdError/sumLoops/abs(max));
			sumdT = 0;
			sumLoops = 0;
			sumError = 0;
			sumdError = 0;
		}
		return out;
	}
	float integrativeError = 0;
	float lastError = 0;

private:
	enum LinguisticLabel {NegativeBig,  NegativeSmall,  NoInput, PositiveSmall, PositiveBig};
	enum OutputLabel {Zero, MediumSmall,Small, Medium, Big, MediumBig, VeryBig};
	typedef  OutputLabel RuleBaseType[5][5];
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

	float fuzzificate (float input, float left, float middle, float right) {
		if ((input >= left) && (input <=middle))
			return (input - left) / (middle - left);
		if ((input >= middle) && (input <=right))
			return (input - middle) / (right - middle);
		return 0;
	}

	float centroidDefuzzification(float fuzzyNumberError[5], float fuzzyNumberdError[5], const RuleBaseType &ruleBase) {
		float fuzzySum = 0;
		float value[5];
		for (int a = 0;a<5;a++) {
			for (int b = 0;b<5;b++) {
				float minFuzzyNumber = min(fuzzyNumberError[a], fuzzyNumberdError[b]);
				if (minFuzzyNumber > 0) {
					OutputLabel output = ruleBase[a][b];
					float v = fuzzificate(minFuzzyNumber, membershipFunction[output][0],membershipFunction[output][1], membershipFunction[output][2]);
					value[output] += v;
					fuzzySum += v;
					logger->print("e/de(");
					logger->print(a);
					logger->print("/");
					logger->print(b);
					logger->print(")->(");
					logger->print((int)output);
					logger->print("/");
					logger->print(v);
				}
			}
		}
		float centroid = 0;
		for (int i = 0;i<5;i++) {
			centroid += value[i]/fuzzySum*(i+1);
		}
		return centroid;
	};

	void updateFuzzy(PIDControllerConfig config, float error, float dError) {
		// fuzzification
		logger->print("fuzz");
		logger->print("e=");
		logger->print(error);
		logger->print("/");
		logger->print(dError);

		float fuzzyNumberError [5];
		fuzzyNumberError[NegativeBig] 		= fuzzificate (error, -1, -1, -0.5);
		fuzzyNumberError[NegativeSmall] 	= fuzzificate (error, -0.75, 0.5, -0.25);
		fuzzyNumberError[NoInput] 			= fuzzificate (error, -0.25, 0, +0.25);
		fuzzyNumberError[PositiveSmall] 	= fuzzificate (error, 0.25, 0.5, +0.75);
		fuzzyNumberError[PositiveBig] 		= fuzzificate (error, 0.5, 1,1);

		float fuzzyNumberdError [5];
		fuzzyNumberdError[NegativeBig] 		= fuzzificate (error, -1, -1, -0.5);
		fuzzyNumberdError[NegativeSmall] 	= fuzzificate (error, -0.75, 0.5, -0.25);
		fuzzyNumberdError[NoInput] 			= fuzzificate (error, -0.25, 0, +0.25);
		fuzzyNumberdError[PositiveSmall] 	= fuzzificate (error, 0.25, 0.5, +0.75);
		fuzzyNumberdError[PositiveBig] 		= fuzzificate (error, 0.5, 1,1);

		logger->print(" e=");
		logger->print(error);
		logger->print(" (");
		for (int i = 0;i<5;i++) {
			if (fuzzyNumberError[i] > 0) {
				logger->print(i);
				logger->print(":");
				logger->print(fuzzyNumberError[i]);
			}
		}
		logger->print(") de=");
		logger->print(dError);
		logger->print(" (");
		for (int i = 0;i<5;i++) {
			if (fuzzyNumberdError[i] > 0) {
				logger->print(i);
				logger->print(":");
				logger->print(fuzzyNumberError[i]);
			}
		}
		logger->print(")");
		logger->print(error);
		logger->print("/");
		logger->print(dError);

		logger->print("KPMOD:");
		float kPModifier = centroidDefuzzification(fuzzyNumberError, fuzzyNumberdError, ruleBaseKp);
		logger->print(" ");
		logger->println(kPModifier);

		logger->print("KIMOD:");
		float kIModifier = centroidDefuzzification(fuzzyNumberError, fuzzyNumberdError, ruleBaseKi);
		logger->print(" ");
		logger->println(kIModifier);
		logger->print("KDMOD:");
		float kDModifier = centroidDefuzzification(fuzzyNumberError, fuzzyNumberdError, ruleBaseKd);
		logger->print(" ");
		logger->print(kDModifier);
		logger->println();

		config.Kp *= kPModifier;
		config.Ki *= kIModifier;
		config.Kd *= kDModifier;

	}

	bool fuzzyOn = false;
	float fuzzydT = 0;
	float sumdT = 0;
	float sumError;
	float sumdError;
	int sumLoops  = 0;
};

/* PID controller that consist of two sets of configuration params.
 * One is used for position control, the other for speed control
 * When pid is running, speed is used to interpolate the PID params
 */
class DynamicPIDController : public PIDController {
public:
	DynamicPIDController () {};
	virtual ~DynamicPIDController () {};

	float update(const PIDControllerConfig &position, const PIDControllerConfig &speed, float min, float max, float speedRatio,
				float error, float dT) {
		float positionRatio = 1.0 - speedRatio;
		PIDControllerConfig config (position.Kp*positionRatio + speed.Kp* speedRatio,
									position.Ki*positionRatio + speed.Ki* speedRatio,
									position.Kd*positionRatio + speed.Kd* speedRatio);
		return PIDController::update(config, error, dT, min, max);
	}
};



#endif /* PIDCONTROLLER_H_ */
