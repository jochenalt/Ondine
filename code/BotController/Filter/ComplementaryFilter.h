/*
 * ComplementaryFilter.h
 *
 * Created: 14.12.2014 13:36:13
 *  Author: JochenAlt
 */


#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "Arduino.h"

class LowPassFilter {
	public:

	LowPassFilter () {
		result = 0;
		compFilterRatio = 0;
	}

	LowPassFilter (float timeDuration, float sampleTime) {
		result = 0;
		init(timeDuration,sampleTime);
	}

	// pass the weight of the most recent value
	// (1-ratio) is the weight of the filtered value
	void init(float timeDuration, float sampleTime) {
		init(timeDuration/(timeDuration + sampleTime));
	};
	void init(float pFilterRatio) {
		compFilterRatio = pFilterRatio;
	};

	// get a filtered value
	float addSample(float pValue) {
		result = compFilterRatio*result + (1.0-compFilterRatio) * pValue;
		return result;
	};

	// get a filtered value
	float get(float pValue) {
		addSample(pValue);
		return get();
	};

	float get() {
		return result;
	}

	void set(float pValue) {
		result = pValue;
	}
	private:
		// complementary value
		float result;
		// weight of the latest value over the last complementary value
		float compFilterRatio;
};
#endif /* COMPLEMENTARYFILTER_H_ */
