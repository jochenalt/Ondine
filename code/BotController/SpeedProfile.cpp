#include "SpeedProfile.h"
#include "libraries/Util.h"

const float floatPrecision = 0.001;

bool almostEqual(float a, float b, float precision) {
	if (a==b)
		return true;
	if (a == 0)
		return (abs(b)<precision);
	if (b == 0)
		return (abs(a)<precision);

	if (b<a)
		return (abs((b/a)-1.0) < precision);
	else
		return (abs((a/b)-1.0) < precision);

}


// abc formular, root of 0 = a*x*x + b*x + c;
bool polynomRoot2ndOrder(float a, float  b, float c, float & root0, float & root1)
{
	float disc = b*b-4.0*a*c;
	if (disc>=0) {
		root0 = (-b + sqrt(disc)) / (2.0*a);
		root1 = (-b - sqrt(disc)) / (2.0*a);
		return true;
	}
	return false;
}

float getDistance(float startSpeed, float acc, float t) {

    float distance = startSpeed*t + 0.5 * acc * sqr(t);
    return distance;
}

SpeedProfile::SpeedProfile() {
	null();
}

SpeedProfile::SpeedProfile(const SpeedProfile& par) {
	operator=(par);
}


void SpeedProfile::operator=(const SpeedProfile& par) {
	startSpeed = par.startSpeed;
	endSpeed = par.endSpeed;
	distance = par.distance;
	duration = par.duration;
	t0 = par.t0;
	t1 = par.t1;
}

bool SpeedProfile::isNull() {
	return ((distance == 0) && (duration == 0));
}

void SpeedProfile::null() {
	startSpeed = 0;
	endSpeed = 0;
	distance = 0.0;
	duration = 0.0;
	t0= 0.0;
	t1= 0.0;
}


bool SpeedProfile::isValid() {
	return isValidImpl(startSpeed, endSpeed, t0,t1, duration, distance);
}


bool SpeedProfile::isValidImpl(float pStartSpeed, float pEndSpeed, float pT0, float pT1, float pDuration, float pDistance) {
	bool valid = almostEqual(computeDistance(pStartSpeed, pEndSpeed, pT0,pT1, pDuration), pDistance,0.01);
	if (valid)
		return true;
	return false;
}

// the rampup profile starts with startSpeed, immediately accelerates to the end speed and remains there. If
// the end speed is lower than the start speed, the speed remains on the start speed as long as possible, then
// goes down to the end speed with maximum negative acceleration.
//
//    |      |           |      |
//    |  ----|v1       v0|----  |
//    | /    |           |    \ |
// v0 |/     |           |     \|v1
//   -|------|-         -|------|---
//
// if the distance is too short to reach the end speed, the end speed is adapted to the maximum resp.
// minimum possible end speed:
//
//    |      /|vmax    v0|\     |
//    |     / |          | \    |
//    |    /  |          |  \   |
//    |   /   |          |   \  |
//    |  /    |          |    \ |
//    | /     |          |     \|
// v0 |/      |          |      |vmin
//   -|-------|-        -|------|---
//
// returns true if end speed has been adapted
bool SpeedProfile::computeRampProfile(const float pStartSpeed, float &pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration) {
	bool endSpeedFine = true;
	// Note: pT1 becomes negative if negative acceleration is to be used
	if (pStartSpeed <= pEndSpeed) {
		// check if endSpeed is possible

		// limit to maximum end speed
		float maxEndSpeed = sqrt( pStartSpeed*pStartSpeed + 2*pDistance*MaxBotAccel);
		if (pEndSpeed > maxEndSpeed+floatPrecision) {
			pEndSpeed = maxEndSpeed;
			endSpeedFine = false;
		}

		pT0= (pEndSpeed - pStartSpeed)/MaxBotAccel;
		pT1 = 0;
		if ((pDistance < floatPrecision) && (pStartSpeed < floatPrecision) && (pEndSpeed < floatPrecision)) {
			pDuration = 0;
		}
		else
			pDuration = (pDistance - pStartSpeed*abs(pT0) - 0.5*MaxBotAccel*sqr(pT0))/pEndSpeed + abs(pT0);
	} else {
		// limit to minimum end speed by distance
		float minEndSpeed = sqrt( pStartSpeed*pStartSpeed - 2*pDistance*MaxBotAccel);
		if (pEndSpeed < minEndSpeed-floatPrecision) {
			pEndSpeed = minEndSpeed;
			endSpeedFine = false;
		}

		pT0 = 0;
		pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel; // becomes negative, since endSpeed < startspeed
		pDuration = (pDistance - pEndSpeed*abs(pT1) - 0.5*MaxBotAccel*sqr(pT1))/pStartSpeed+ abs(pT1);
	}
	return endSpeedFine;
}

bool SpeedProfile::getRampProfileDuration(float& pStartSpeed, float& pEndSpeed, float pDistance, float &pDuration) {
	float t0, t1;
	pDuration = 0;
	bool endSpeedFine = computeRampProfile(pStartSpeed, pEndSpeed, pDistance, t0, t1, pDuration);
	return endSpeedFine;
}

// compute duration if we stay as long as possible at startspeed
void  SpeedProfile::getLazyRampProfileDuration(const float pStartSpeed, const float pEndSpeed, float pDistance, float &pDuration) {
	// what is the duration that would make t0 = 0 ?
	pDuration = (pDistance - 0.5*sqr(pEndSpeed-pStartSpeed)/MaxBotAccel)/pStartSpeed;
}



// the trapezoid profile starts with startSpeed, immediately accelerates to the middle speed and deccelerates to the
// end speed at the latest point in time such that the given distance is met.
//
//    |       |           |      |
//    |  ---- |         v0| ---  |
//    | /    \|v1         |/   \ |
// v0 |/      |           |     \|v1
//   -|-------|-         -|------|---
//
bool SpeedProfile::computeTrapezoidProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration) {
	float u = (pStartSpeed-pEndSpeed)/MaxBotAccel;
	// abc formula
	float c = pStartSpeed*pDuration - 0.5*MaxBotAccel*sqr(u) - pDistance;
	float b = MaxBotAccel * ( pDuration - u );
	float a = -MaxBotAccel;
	float t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel - pT0;
	return true;
}

// A negative traezoid profile is returned
//
// v0 |       |           |      |
//    |\      |v1         |     /|v1
//    | \    /|         v0|\   / |
//    |  ---  |           |  --  |
//   -|-------|-         -|------|---
//
// returns true, if solution exists
bool SpeedProfile::computeNegativeTrapezoidProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration) {
	float u = (pStartSpeed-pEndSpeed)/MaxBotAccel;
	// abc formula
	float c = pStartSpeed*pDuration + 0.5*MaxBotAccel*sqr(u) - pDistance;
	float b = MaxBotAccel * ( pDuration + u );
	float a = MaxBotAccel;
	float t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel - pT0;
	return true;
}


// the peak profile starts with startSpeed, immediately accelerates and deccelerates to the
// end speed such that the given distance is met.
//
//    |   /\  |
//    |  /  \ |
//    | /    \|v1
// v0 |/      |
//   -|-------|-
//
//
// returns true if solution exists
bool SpeedProfile::computePeakUpProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration) {
	float u = (pStartSpeed-pEndSpeed)/MaxBotAccel;
	// abc formula
	float c = pEndSpeed*u + 0.5*MaxBotAccel*sqr(u) - pDistance;
	float b = (pStartSpeed + pEndSpeed + MaxBotAccel * u);
	float a = MaxBotAccel;
	float t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel - pT0;

	// ignore input of duration, overwrite
	pDuration = abs(pT0) + abs(pT1);
	return true;
}

// the peak profile starts with startSpeed, immediately accelerates and deccelerates to the
// end speed such that the given distance is met.
//
//       |     |
//     v0|\    |
//       | \  /|
//       |  \/ |v1
//      -|-----|---
//
//
// returns true if solution exists
bool SpeedProfile::computePeakDownProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration) {
	float u = (pStartSpeed-pEndSpeed)/MaxBotAccel;
	// abc formula
	// TODO check formula
	float c = -pEndSpeed*u - 0.5*MaxBotAccel*sqr(u) - pDistance;
	float b = (pStartSpeed + pEndSpeed - MaxBotAccel * u);
	float a = -MaxBotAccel;
	float t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel - pT0;

	// ignore input of duration, overwrite
	pDuration = abs(pT0) + abs(pT1);
	return true;
}

// the stairways profile starts with startSpeed, immediately accelerates to the middle speed and deccelerates to the
// end speed at the latest point in time such that the given distance is met.
//
//    |      /|v1       v0|\     |
//    |  ---/ |           | \--  |
//    | /     |           |    \ |
// v0 |/      |           |     \|v1
//   -|-------|-         -|------|---
//
void SpeedProfile::computeStairwaysProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration) {
	pT0= (pDistance - pStartSpeed*pDuration - 0.5 * sqr(pEndSpeed - pStartSpeed)/MaxBotAccel )
				/
				(MaxBotAccel * pDuration + pStartSpeed - pEndSpeed);
	pT1 = (pEndSpeed - pStartSpeed)/MaxBotAccel - pT0;
}

bool SpeedProfile::computeSpeedProfileImpl(float& pStartSpeed, float& pEndSpeed, float pDistance, float& pT0, float& pT1, float& pDuration) {
	if (pStartSpeed <= pEndSpeed) {

	   // compute ramp profile to decide which profile to use
	   float rampDuration;
	   bool endEndSpeedFine= computeRampProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (!endEndSpeedFine) {
		   // even with max acceleration, end speed cannot be reached, it has been modified by computeRampProfile
		   pDuration = rampDuration; // overwrite input of duration
		   return false; // adaption was necessary
	   }

	   // if the to-be duration is smaller than the ramp's duration, we need to speed up to a trapezoid profile
	   if (pDuration < rampDuration) {
		   // trapezoid profile is possible if duration is longer than peakDuration
		   float peakDuration;
		   bool peakExists = computePeakUpProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, peakDuration);
		   if (peakExists) {
			   if (pDuration >= peakDuration) { // trapezoid profile possible
				   bool trapezoidSolutionValid = computeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (trapezoidSolutionValid) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
						   fatalError("BUG: trapezoid profile invalid");
					   return true; // everything fine
				   } else
					   fatalError("BUG: solution for trapezoid profile expected but not found");
			   } else {
				   // trapezoid profile not possible, adapt to peak profile
				   pDuration = peakDuration; // overwrite input of duration
				   return false; // amendment happened
			   }
		   } else
			   fatalError( "BUG:peak profile solution expected");
	   } else {  // pDuration >= rampDuration
		   // we have more time than ramp profile. We need to slow down,
		   // either with stairways or negative trapezoid profile
		   if (pDuration > rampDuration) {
			   // check if stairways or neg. trapezoid by comparing duration with lazy ramp profile
			   // (lazy ramp = ramp that stays as long as possible on startspeed)
			   float lazyrampDuration;
			   getLazyRampProfileDuration(pStartSpeed, pEndSpeed, pDistance, lazyrampDuration);
			   if (pDuration < lazyrampDuration) {
				   // use stairways profile
				   computeStairwaysProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
					   fatalError("BUG: stairways profile invalid");
				   return true; // everything fine
			   }
			   else {
				   // use negative trapezoid profile
				   bool solutionExists= computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (solutionExists) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed,pT0,pT1, pDuration, pDistance))
						   fatalError("BUG: negative trapezoid profile invalid");
					   return true; // everything fine
				   } else
					   fatalError("BUG: solution for negative trapezoid profile expected ");
			   }
	   	   } else {
	   		   // fits exactly into a ramp profile. Very unlikely
			   pDuration = rampDuration;
			   return true; // nothing changed
	   	   }
	   }
   } else { // pStartSpeed > pEndspeed
	   // compute ramp profile to decide which profile to use
	   float rampDuration;
	   bool noEndSpeedAmendment = computeRampProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (!noEndSpeedAmendment) {
		   // even with max decceleration, end speed cannot be reached, so amend end speed to minimum
		   pDuration = rampDuration; // overwrite input of duration
		   return false; // adaption was necessary
	   }

	   // if the to-be duration is smaller than the ramp's duration, we need to speed up to a trapezoid profile
	   if (pDuration < rampDuration-floatPrecision) {
		   // trapezoid profile is possible if duration is longer than peakDuration
		   float peakDuration;
		   bool peakExists = computePeakUpProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, peakDuration);
		   if (peakExists) {
			   if (pDuration >= peakDuration) { // trapezoid profile possible
				   bool trapezoidSolutionValid = computeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (trapezoidSolutionValid) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
						   fatalError("BUG: neg trapezoid profile invalid");
					   return true; // everything fine
				   } else
					   fatalError("BUG: solution for negative trapezoid profile expected but not found");
			   } else {
				   // trapezoid profile not possible, adapt to peak profile
				   pDuration = peakDuration; // overwrite input of duration
				   return false; // amendment happened
			   }
		   } else
			   fatalError("BUG:peak down profile solution expected");
	   } else {
		   // we have more time than ramp profile. We need to slow down,
		   // either with stairways or negative trapezoid profile
		   if (pDuration > rampDuration+floatPrecision) {
			   // check if stairways or neg. trapezoid by comparing duration with lazy ramp profile
			   // (lazy ramp = ramp that stays as long as possible on startspeed)
			   float lazyrampDuration;
			   getLazyRampProfileDuration(pStartSpeed, pEndSpeed, pDistance, lazyrampDuration);
			   if (pDuration < lazyrampDuration) {
				   // use stairways profile
				   computeStairwaysProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
					   fatalError("BUG: stairways profile invalid");
				   return true; // everything fine
			   }
			   else {
				   // use negative trapezoid profile
				   bool solutionExists= computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (solutionExists) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed,pT0,pT1, pDuration, pDistance))
						   fatalError("BUG: negative trapezoid profile invalid");
					   return true; // everything fine
				   } else
					   fatalError("BUG: solution for negative trapezoid profile expected ");
			   }
	   	   } else {
	   		   // fits exactly into a ramp profile. Very unlikely
			   pDuration = rampDuration;
			   return true; // nothing changed
	   	   }
	   }
   }
	fatalError( "BUG:no speed profile found");
   return false;
}



bool SpeedProfile::computeSpeedProfile(float& pStartSpeed, float& pEndSpeed, float pDistance, float& pDuration) {
	t0 = 0.0;
	t1 = 0.0;
	bool possibleWithoutAmendments = computeSpeedProfileImpl(pStartSpeed, pEndSpeed, pDistance, t0, t1, pDuration);
	distance = pDistance;
	duration = pDuration;
	startSpeed = pStartSpeed;
	endSpeed = pEndSpeed;

	if (!isValid())
		fatalError("returned speed profile invalid");
	return possibleWithoutAmendments;
}


float SpeedProfile::computeDistance(float pStartSpeed, float pEndSpeed, float pT0, float pT1, float pDuration) {
	float middleSpeed = pStartSpeed + MaxBotAccel * pT0;
	float position =
			getDistance(pStartSpeed, sgn(pT0)* MaxBotAccel, fabs(pT0))
				+ getDistance(middleSpeed, 0,pDuration - fabs(pT0) - fabs(pT1))
				+ getDistance(middleSpeed, sgn(pT1) * MaxBotAccel, fabs(pT1));
	return position;
}

float SpeedProfile::getDistanceSoFar(float t0, float t1, float t) {
	float middleSpeed = startSpeed + MaxBotAccel * t0;

	float absT = t*duration;
	float position = 0;

	float durationFirstBlock = fabs(t0);
	float durationThirdBlock = fabs(t1);
	float durationSecondBlock = duration - durationFirstBlock - durationThirdBlock;

	if (absT < fabs(t0)) {
		// first part, increase speed from startSpeed to middleSpeed with constant acceleration
		float tInBlock = absT;
		position = getDistance(startSpeed, sgn(t0)* MaxBotAccel, tInBlock);
	} else {
		if (absT < (durationFirstBlock+durationSecondBlock)) {
			// second part, constant middleSpeed
			float tInBlock = absT - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* MaxBotAccel, durationFirstBlock)
 					 + getDistance(middleSpeed, 0, tInBlock);
		} else {
			// third part, increase speed from middleSpeed to endSpeed;
			float tInBlock = absT - durationSecondBlock - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* MaxBotAccel, durationFirstBlock)
						+ getDistance(middleSpeed, 0,durationSecondBlock)
						+ getDistance(middleSpeed, sgn(t1) * MaxBotAccel, tInBlock);
		}
	}

	return position;
}


// compute distance by given speed and acceleration
float SpeedProfile::apply(SpeedProfileType type, float t) {
	if (isNull() || (type == LINEAR))
		return t;

	float distanceSoFar = getDistanceSoFar(t0,t1,t);
	float result = distanceSoFar/distance;

	if ((result >= 1.0) && (result < 1.0 + sqrt(floatPrecision)))
		result = 1.0;

	if ((result < 0.0) || (result > 1.0)) {
		logger->print("BUG: speedprofile (");
		logger->print(t);
		logger->print(" returns t=");
		logger->print(result);

	}
	return result;
}


