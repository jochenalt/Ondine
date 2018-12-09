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

KalmanFilter::KalmanFilter() {
	setup(0);
};

void KalmanFilter::setup(float angle) {
    Q_angle = 0.001f;	// default 0.001
    Q_bias = 0.003f;	// default 0.003
    R_measure = 0.03;	// default 0.03

    this->angle = angle;
    bias = 0.0f;

    P00 = 0.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.0f;
}

void KalmanFilter::setNoiseVariance(float noiseVariance) {
    R_measure = noiseVariance;	// default 0.03
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
void KalmanFilter::update(float newAngle /* rad */, float newRate /* rad/s */, float dt) {
    // predict the state after dT
    rate = newRate - bias;
    angle += dt * rate;

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
    angle += K0 * y;
    bias  += K1 * y;

    // Update the error covariance
    // (strange: most implementations in the internet forget to save the variables before using it)
    float P00saved = P00;
    float P01saved= P01;

    P00 -= K0 * P00saved;
    P01 -= K0 * P01saved;
    P10 -= K1 * P00saved;
    P11 -= K1 * P01saved;
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
	return this->rate;
};

void KalmanFilter::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void KalmanFilter::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void KalmanFilter::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float KalmanFilter::getQangle() { return this->Q_angle; };
float KalmanFilter::getQbias() { return this->Q_bias; };
float KalmanFilter::getRmeasure() { return this->R_measure; };
