/*
 * PIDController.cpp
 *
 *  Created on: 29.08.2018
 *      Author: JochenAlt
 */

#include <libraries/PIDController.h>
#include <types.h>
const float OneMicrosecond_s = 0.000001;

void PIDController::reset() {
	integrativeError = 0;
	lastError = 0;
}

float PIDController::update (const PIDControllerConfig& params, float error, float dT, float min, float max) {
	if ((dT > floatPrecision) && (dT < 1.0)) {
		float pOut = params.Kp*error;
		integrativeError += error * dT;
		integrativeError = constrain(integrativeError, min, max);
		float iOut = params.Ki* integrativeError;
		float dError = error - lastError;
		float dOut  = 0;
		if (dT > OneMicrosecond_s)
			dOut = params.Kd * dError / dT;
		float out = pOut + iOut + dOut;
		out = constrain(out, min, max);
		lastError = error;

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
	return 0;
}

