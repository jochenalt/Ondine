/*
 * ComplementaryFilter.h
 *
 * Created: 14.12.2014 13:36:13
 *  Author: JochenAlt
 */


#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "Arduino.h"
#include "CircularDelay.h"
#include "libraries/Util.h"

enum FilterType {Exponentional, Averaging, Frequency};

class LowPassFilterAverage {
	public:

	LowPassFilterAverage () {
		init(0);
	}

	LowPassFilterAverage (int noOfPoints) {
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


class LowPassFilter2ndOrder{
public:

	LowPassFilter2ndOrder() {
	}

	void init(float cutOffFrequency, float sampleFrequency) {
		float tau = 1.0/(2*M_PI*cutOffFrequency);
		float dt = 1.0 / sampleFrequency;
		initInternal(dt, tau);
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
	void initInternal(float dt, float tau_c) {
		yc[0] = -2 * (pow(dt, 2) - 4 * pow(tau_c, 2)) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2));
		yc[1] = (-pow(dt, 2) + 2 * sqrt(2) * tau_c * dt - 4 * pow(tau_c, 2)) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2));
		xc[0] = pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2));
		xc[1] = 2 * pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2));
		xc[2] = pow(dt, 2) / (pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * pow(tau_c, 2));
		if(tau_c < M_PI * dt){
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
		x.push(newValue);
		float output = 0;
		for (int i = 0; i < 2; ++i)
			output += y.get(i) * yc[i];

		for (int i = 0; i < 3; ++i)
			output += x.get(i) * xc[i];
		return y.push(output);
	}
	/**
	 * @brief      Gets the output.
	 *
	 * @return     The output.
	 */
	float get() {return y.get(0);}
	/**
	 * @brief      Force the output to a desired value
	 *
	 *             This can be useful when the output needs to be forced in case
	 *             of extreme inputs or such
	 *
	 * @param[in]  newOutput  The new output
	 */
	void set(float newOutput){
		for(auto& it : x){
			it = newOutput;
		}
		for(auto& it : y){
			it = newOutput;
		}
	}
private:
	float yc[2] = {0,0};
	float xc[3] = {0,0,0};
	CircularDelay<float, 2> y;
	CircularDelay<float, 3> x;
};

class LowPassFilter1stOrder {
public:
	LowPassFilter1stOrder() {

	};

	void init(float cutOffFrequency, float sampleFrequency) {
		float tau = 1.0/(2.0*M_PI*cutOffFrequency);
		float omega = 1.0/tau;;
		float dt = 1.0 / sampleFrequency;
		logging("sampleFrequency");
		loggingln(sampleFrequency);

		logging("dt");
		loggingln(dt,3);
		logging("omega");
		loggingln(omega,3);

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
		logging("epow");
		loggingln(epow,3);

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

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the bilinear transform.
 */
class LowPass3rdOrder {
public:
	LowPass3rdOrder () {};

	void init(float cutOffFrequency, float sampleFrequency) {
		float tau = 1.0/(cutOffFrequency);
		float omega = 1.0/tau;

		float dt = 1.0 / sampleFrequency;
		initInternal(dt, omega);
	}

	void initInternal(float sampleTime, float omega_c) {
		yc[0] = 1;
		yc[1] =
			(float)((3 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c - 24)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		yc[2] =
			(float)((3 * pow(sampleTime * omega_c, 3) - 4 * pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c + 24)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		yc[3] =
			(float)((1 * pow(sampleTime * omega_c, 3) - 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c - 8)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		xc[0] =
			(float)(1 * pow(sampleTime * omega_c, 3)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		xc[1] =
			(float)(3 * pow(sampleTime * omega_c, 3)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		xc[2] =
			(float)(3 * pow(sampleTime * omega_c, 3)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		xc[3] =
			(float)(1 * pow(sampleTime * omega_c, 3)
			/
			(1 * pow(sampleTime * omega_c, 3) + 4 * pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8));
		if(omega_c < sampleTime){
			fatalError("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
		}
		}
	float update(float newValue) {
		x.push(newValue);
		double y0 = 0;
		const float* doubleP = xc;
		for (auto it = x.rbegin(); it != x.rend(); it++)
		{
			y0 += *it * *doubleP++;
		}
		doubleP = yc + 1;
		for (auto it = y.rbegin(); it != y.rend(); it++)
		{
			y0 -= *it * *doubleP++;
		}
		return y.push(yc[0] * y0);
	}
	float get() {return y.get(0);}
private:
	float yc[4];
	float xc[4];
	CircularDelay<float, 3> y;
	CircularDelay<float, 4> x;
};

#endif /* COMPLEMENTARYFILTER_H_ */
