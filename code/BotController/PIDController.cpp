/*
 * PIDController.cpp
 *
 *  Created on: 29.08.2018
 *      Author: JochenAlt
 */

#include <PIDController.h>

float PIDController::update (PIDControllerConfig& params, float error, float dT, float min, float max) {
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

		// carry out the self-tuning fuzzy adaption
		// do this not everytime (computing intense) but with a rate of fuzzydT set in turnFuzzyAdaption
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

	float PIDController::triangleFunction (float input, float left, float middle, float right) {
		if ((input >= left) && (input <=middle))
			return (input - left) / (middle - left);
		if ((input >= middle) && (input <=right))
			return (input - middle) / (right - middle);
		return 0;
	}

	float PIDController::centroidDefuzzification(float membershipRatioError[5], float membershipRatioDError[5], const RuleBaseType &ruleBase) {
		float fuzzySum = 0;
		float value[5];
		float middle[5];

		for (int errorLabelIdx = 0;errorLabelIdx<5;errorLabelIdx++) {
			for (int derrorLabelIdx = 0;derrorLabelIdx<5;derrorLabelIdx++) {
				// rules have the form if e=x AND de=y, the AND is implemented as minimum
				float outcomeFuzzyNumber = min(membershipRatioError[errorLabelIdx], membershipRatioDError[derrorLabelIdx]);
				if (outcomeFuzzyNumber > 0) {
					// apply rule
					OutputLabel ruleOutcome = ruleBase[derrorLabelIdx][errorLabelIdx];
					float v = triangleFunction(outcomeFuzzyNumber, membershipFunction[ruleOutcome][0] /* left */,membershipFunction[ruleOutcome][1] /* middle */, membershipFunction[ruleOutcome][2] /* right */);
					value[ruleOutcome] += v;
					middle[ruleOutcome] = membershipFunction[ruleOutcome][1];
					fuzzySum += v;
					logger->print("de/e(");
					logger->print(getLinguisticLabelName((LinguisticLabel)derrorLabelIdx));
					logger->print(",");
					logger->print(getLinguisticLabelName((LinguisticLabel)errorLabelIdx));
					logger->print("(");
					logger->print(outcomeFuzzyNumber);
					logger->print("))->(");
					logger->print(getOutputLabelName(ruleOutcome));
					logger->print("(");
					logger->print(v);
					logger->print("))");
				}
			}
		}
		logger->print("fuzzySum=");
		logger->print(fuzzySum);

		// approximate the centroid of all rule outcomes by taking the average of all middle positions weighted with their membership ratio
		float centroid = 0;
		for (int i = 0;i<5;i++) {
			centroid += middle[i] * (value[i]/fuzzySum);
		}
		return centroid;
	};

	void PIDController::updateFuzzy(PIDControllerConfig config, float error /* normalized from -1..+1 */, float dError /* normalized from -1..+1 */) {
		if (abs(error) > 1) {
			logger->print("ERR:error ");
			logger->println(error);
		}
		if (abs(dError) > 1) {
			logger->print("ERR:derror ");
			logger->println(dError);
		}

		// fuzzification
		logger->print("fuzz:");

		float membershipRatioError [5];
		membershipRatioError[NegativeBig] 		= triangleFunction (error, -1, -1, -0.5);
		membershipRatioError[NegativeSmall] 	= triangleFunction (error, -0.75, 0.5, -0.25);
		membershipRatioError[NoInput] 			= triangleFunction (error, -0.25, 0, +0.25);
		membershipRatioError[PositiveSmall] 	= triangleFunction (error, 0.25, 0.5, +0.75);
		membershipRatioError[PositiveBig] 		= triangleFunction (error, 0.5, 1,1);

		float membershipRatioDError [5];
		membershipRatioDError[NegativeBig] 		= triangleFunction (error, -1, -1, -0.5);
		membershipRatioDError[NegativeSmall] 	= triangleFunction (error, -0.75, 0.5, -0.25);
		membershipRatioDError[NoInput] 			= triangleFunction (error, -0.25, 0, +0.25);
		membershipRatioDError[PositiveSmall] 	= triangleFunction (error, 0.25, 0.5, +0.75);
		membershipRatioDError[PositiveBig] 		= triangleFunction (error, 0.5, 1,1);

		logger->print(" e=");
		logger->print(error);
		logger->print(" (");
		for (int i = 0;i<5;i++) {
			if (membershipRatioError[i] > 0) {
				logger->print(getLinguisticLabelName((LinguisticLabel)i));
				logger->print("(");
				logger->print(membershipRatioError[i]);
				logger->print(")");
			}
		}
		logger->print(") de=");
		logger->print(dError);
		logger->print(" (");
		for (int i = 0;i<5;i++) {
			if (membershipRatioDError[i] > 0) {
				logger->print(getLinguisticLabelName((LinguisticLabel)i));
				logger->print("(");
				logger->print(membershipRatioError[i]);
				logger->print(")");
			}
		}
		logger->print(")");
		logger->print(error);
		logger->print("/");
		logger->print(dError);

		logger->print("KPMOD:");
		float kPModifier = centroidDefuzzification(membershipRatioError, membershipRatioDError, ruleBaseKp);
		logger->print(" ");
		logger->println(kPModifier);

		logger->print("KIMOD:");
		float kIModifier = centroidDefuzzification(membershipRatioError, membershipRatioDError, ruleBaseKi);
		logger->print(" ");
		logger->println(kIModifier);
		logger->print("KDMOD:");
		float kDModifier = centroidDefuzzification(membershipRatioError, membershipRatioDError, ruleBaseKd);
		logger->print(" ");
		logger->print(kDModifier);
		logger->println();

		config.Kp *= kPModifier;
		config.Ki *= kIModifier;
		config.Kd *= kDModifier;

	}
