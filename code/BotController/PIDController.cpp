/*
 * PIDController.cpp
 *
 *  Created on: 29.08.2018
 *      Author: JochenAlt
 */

#include <PIDController.h>

void PIDController::reset() {
	integrativeError = 0;
	lastError = 0;
}

float PIDController::update (PIDControllerConfig& params, float error, float dT, float min, float max) {
		float pOut = params.Kp*error;
		integrativeError += error * dT;
		// integrativeError = constrain(integrativeError, min, max);
		float iOut = params.Ki* integrativeError;
		float dError = error - lastError;
	    float derivative = dError / dT;
	    double dOut = params.Kd * derivative;
		lastError = error;
		float out = pOut + iOut + dOut;
		out = constrain(out, min, max);
/*
		logger->print("pid(");
		logger->print(min);
		logger->print("/");
		logger->print(max);
		logger->print(" e=");

		logger->print(error);

		logger->print(error);
		logger->print(" ");
		logger->print(pOut);
		logger->print(" ");
		logger->print(iOut);
		logger->print(" ");
		logger->print(dOut);
		logger->print("=");
		logger->print(out);
		logger->println(")");

*/
		return out;
}

