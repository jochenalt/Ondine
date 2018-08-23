/*
 * Kalman.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */
#include "Kalman.h"

Kalman::Kalman() {
	setup(0);
};

void Kalman::setup(float angle) {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    this->angle = angle;
    bias = 0.0f;

    P00 = 0.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::update(float newAngle, float newRate, float dt) {

    // predict the state after dT
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - project the error covariance ahead
    P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    // Compute the Kalman gain
    float S = P00 + R_measure; // Estimate error
    float K0, K1; // Kalman gain - This is a 2x1 vector
    K0 = P00 / S;
    K1 = P10 / S;

    // Update estimate with measurement zk (newAngle)
    float y = newAngle - angle; // Angle difference
    angle += K0 * y;
    bias  += K1 * y;

    // Update the error covariance
    float P00_temp = P00;
    float P01_temp = P01;

    P00 -= K0 * P00_temp;
    P01 -= K0 * P01_temp;
    P10 -= K1 * P00_temp;
    P11 -= K1 * P01_temp;

    return angle;
};

float Kalman::getAngle() {
	return angle;
}

// Used to set angle, this should be set as the starting angle
void Kalman::setAngle(float angle) {
	this->angle = angle;
};

// Return the unbiased rate
float Kalman::getRate() {
	return this->rate;
};

void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };
