/*
 * SpeedProfile.h
 *
 * Implementation of a linear and a trapezoidal speed profile.
 * Computed between two points defined by start speed, end speed, distance and to-be
 * duration that is used to move from a to b.
 *
 * Author: JochenAlt
 */

#ifndef SPEEDPROFILE_H_
#define SPEEDPROFILE_H_

#include "setup.h"

typedef float mmPerMillisecond;
class SpeedProfile {
public:
	enum SpeedProfileType {LINEAR, TRAPEZOIDAL };

	SpeedProfile();
	SpeedProfile(const SpeedProfile& par);
	void operator=(const SpeedProfile& par);
	bool isNull();
	void null();

	// compute a trapezoidal speed profile. Depending on startspeed/endspeed/distance,
	// select the appropriate shape and return the duration
	bool computeSpeedProfile(float& pStartSpeed /* mm/s */, float& pEndSpeed /* mm/s */, float pDistance /* mm */, float& pDuration /* ms */);

	// returns true, if a ramp profile can be achieved with given startspeed/endspeed/distance. If yes, the duration is computed.
	static bool getRampProfileDuration(float& pStartSpeed, float& pEndSpeed, float pDistance, float &pDuration);

	// return true, if profile is possible. if invalid, profile is set to null, which is an linear profile
	bool isValid();

	// use a valid speed profile by modifying t=[0..1] such the
	// returned number [0..1] can be used as input parameter
	// representing the position within a trajectory piece.
	float apply(SpeedProfileType type, float t);

private:
	static bool computeSpeedProfileImpl(float& pStartSpeed, float& pEndSpeed, float pDistance, float& pT0, float& pT1, float& pDuration);
	static bool computeRampProfile(const float pStartSpeed, float& pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration);
	static void getLazyRampProfileDuration(const float pStartSpeed, const float pEndSpeed, float pDistance, float &pDuration);
	static void computeStairwaysProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration);
	static bool computeTrapezoidProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration);
	static bool computeNegativeTrapezoidProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, const float pDuration);
	static bool computePeakUpProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration);
	static bool computePeakDownProfile(const float pStartSpeed, const float pEndSpeed, const float pDistance, float& pT0, float& pT1, float& pDuration);

	static float computeDistance(float pStartSpeed, float pEndSpeed, float pT0, float pT1, float pDuration);
	float getDistanceSoFar(float t0, float t1, float t);
	static bool isValidImpl(float pStartSpeed, float pEndSpeed, float pT0, float pT1, float pDuration, float pDistance);


	mmPerMillisecond startSpeed;
	mmPerMillisecond endSpeed;
	float distance;
	float duration;
	float t0;		 // time diff of first phase (negative when going down)
	float t1;		 // time diff of last phase (also might be negative)
};

#endif /* SPEEDPROFILE_H_ */
