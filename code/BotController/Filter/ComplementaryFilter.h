/*
 * ComplementaryFilter.h
 *
 * Created: 14.12.2014 13:36:13
 *  Author: JochenAlt
 */


#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "Arduino.h"
#include "libraries/Util.h"

class Average {
	public:

	Average () {
		init(0);
	}

	Average (int taps) {
		init(0);
	}

	// get a filtered value
	void update(float input) {
		if (NoOfSamples >= MaxNoOfSamples) {
			// delete first sample
			for (int i = 0;i<MaxNoOfSamples-1;i++)
				samples[i] = samples[i+1];
			NoOfSamples--;
		}
		samples[NoOfSamples++] = input;
	};

	float get() {
		if (NoOfSamples > 0) {
			float sum = 0;
			for (int i = 0;i<NoOfSamples;i++)
				sum += samples[i];
			result = sum/NoOfSamples;
			NoOfSamples = 0;
		}
		return result;
	}

	void init(int points) {
		this->NoOfSamples = points;
		this->result = 0;
	}

private:
	// complementary value
	float result = 0;
	int NoOfSamples = 0;
	// weight of the latest value over the last complementary value
	const static int MaxNoOfSamples = 64;
	float samples[MaxNoOfSamples];
};


class LowPassFilterMovingAverage {
	public:

	LowPassFilterMovingAverage () {
		init(0);
	}

	LowPassFilterMovingAverage (int noOfPoints) {
		init(noOfPoints);
	}

	// get a filtered value
	float update(float input) {
		float sum = 0;
	    for(int i = points - 1; i < 0; --i)
	    {
	    	samples[i] = samples[i-1];
	        sum += samples[i];
	    }
	    sum += input;
	    samples[0] = input;
	    result = sum/points;
	    return result;
	};

	// get a filtered value
	float get(float pValue) {
		update(pValue);
		return get();
	};

	float get() {
		return result;
	}

	void init(int points) {
		this->points = points;
		for (int i = 0;i<MaxNoOfTaps;i++) {
			samples[i] = 0;
		}
		result = 0;
	}
	private:
		// complementary value
		float result;
		int points;
		// weight of the latest value over the last complementary value
		const static int MaxNoOfTaps = 16;
		float samples[MaxNoOfTaps];
};


class LowPassFilterFrequency{
	public:

	LowPassFilterFrequency () {
		init(0, 0);
	}

	LowPassFilterFrequency (float cutOffFrequency, float sampleFrequency) {
		init(cutOffFrequency, sampleFrequency);
	}

	float update(float input) {
		result = (1.0-alpha)*result + alpha*input;
		return result;
	};

	// get a filtered value
	float get(float pValue) {
		update(pValue);
		return get();
	};

	float get() {
		return result;
	}

	void init(float cutOffFrequency, float sampleFrequency) {
		this->cutOffFrequency = cutOffFrequency;
		this->sampleFrequency = sampleFrequency;
		float RC = 1.0/(cutOffFrequency*2.0*M_PI);
		float dt = 1.0/sampleFrequency;
		alpha = dt/(RC+dt);
		result = 0;
	}
	private:
		// complementary value
		float result;
		float cutOffFrequency;
		float sampleFrequency;
		float alpha;
};


class LowPassFilter1stOrder {
public:
	LowPassFilter1stOrder() {

	};

	void init(float cutOffFrequency, float sampleFrequency) {
		float tau = 1.0/(2.0*M_PI*cutOffFrequency);
		float omega = 1.0/tau;;
		float dt = 1.0 / sampleFrequency;
		initInternal(dt, omega);
	}
	/**
	 * @brief      Constructor to set sample time and the tau constant
	 *
	 * @param[in]  idt     Sample time for the low pass filter
	 * @param[in]  itua_c  Or
	 *             @f$ \tau_c
	 *             @f$ The time constant for the filter. Note that
	 *             @f$ \tau_c = \frac{1}{2 pi f_c}@f$  where @f$ f_c @f$ is the cutoff frequency
	 */
	void initInternal(float idt, float omega_c) {
		epow = exp(-idt * omega_c);
		output = 0;
		if(omega_c < idt){
				fatalError("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
		}
	}
	/**
	 * @brief      Update function to push new value into the low pass filter
	 *
	 * @param[in]  newValue  The new value after dt time
	 *
	 * @return     The new output value
	 */
	float update(float newValue) {
		output = output*epow + newValue*(1-epow);
		return output;
	}
	/**
	 * @brief      Gets the output.
	 *
	 * @return     The output.
	 */
	float get() {return output;}
	/**
	 * @brief      Force the output to a desired value
	 *
	 *             This can be useful when the output needs to be forced in case
	 *             of extreme inputs or such
	 *
	 * @param[in]  newOutput  The new output
	 */
	void set(float newOutput){output = newOutput;}
private:
	float epow = 0; /// one time calculation constant
	float output = 0;
};

#endif /* COMPLEMENTARYFILTER_H_ */
