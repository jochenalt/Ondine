/*
 * Kalman.cpp
 *
 * Efficient implementation of a Kalman filter
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */
#include <Arduino.h>
#include <Filter/KalmanFilter.h>
#include <libraries/Util.h>

KalmanFilter::KalmanFilter() {
	setup(0);
};

void KalmanFilter::setup(float angle) {
    Q_angle = 0.001f;	// default 0.001
    Q_bias = 0.002f;	// default 0.003
    R_measure = 0.02;	// default 0.03

    this->angle = angle;
    bias = 0.0f;

    P00 = 0.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.0f;
}

void KalmanFilter::setNoiseVariance(float noiseVariance) {
    R_measure = noiseVariance;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
void KalmanFilter::update(float newAngle /* [rad] */, float newRate /* [rad/s] */, float dt /* [s] */) {
    // predict the state after dT
    rate = newRate - bias;
    angle += dt * rate;
    gyroIntegrated += dt*rate;

    // update estimation error covariance
    P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // Kalman gain - This is a 2x1 vector
    float S =  P00 + R_measure; // Estimate error
    float K0 = P00 / S;
    float K1 = P10 / S;

    // update estimate with passed measurement
    float y = newAngle - angle; // Angle difference
    angle += K0 * y; 			// add kalman gain of angle difference to angle
    bias  += K1 * y;			// add kalman gain of angle different to gyro bias

    /*
    static int c = 2;
    if (c == 2) {
    	c = 0;
		logging("a=");
		logging(degrees(newAngle),2);
		logging(" a'=");
		logging(degrees(newRate),2);
		logging(" na=");
		logging(degrees(angle),2);
		logging(" na'=");
		logging(degrees(rate),2);
		logging(" bias=");
		logging(degrees(bias),2);
		logging(" K'=");
		logging(K0,4);
		logging(",");
		logging(K1,4);
		logging(" t=");
		logging(degrees((angle - oldAngle)/dt),4);
		logging(" Sg=");
		logging(degrees(gyroIntegrated),4);
		logging(" Sgb=");
		loggingln(degrees(gyroIntegrated-bias),4);

    } else
    	c++;
    */

    // Update the error covariance
    P00 -= K0 * P00;
    P01 -= K0 * P01;
    P10 -= K1 * P00;
    P11 -= K1 * P01;

};

float KalmanFilter::getAngle() {
	return angle;
}

// Used to set angle, this should be set as the starting angle
void KalmanFilter::setAngle(float angle) {
	this->angle = angle;
};

// Return the unbiased rate
float KalmanFilter::getRate() {
	return rate;
};

void KalmanFilter::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void KalmanFilter::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void KalmanFilter::setRmeasure(float R_measure) { this->R_measure = R_measure; };
float KalmanFilter::getQangle() { return this->Q_angle; };
float KalmanFilter::getQbias() { return this->Q_bias; };
float KalmanFilter::getRmeasure() { return this->R_measure; };
