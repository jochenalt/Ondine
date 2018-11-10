/*
 * Kalman.h
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#ifndef KALMAN_KALMAN_H_
#define KALMAN_KALMAN_H_

class KalmanFilter {
public:
    KalmanFilter();
    virtual ~KalmanFilter() {};

    void setup(float angle);

    void setNoiseVariance(float noiseVariance);

    // Set the starting angle
    void setAngle(float angle);

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    void update(float newAngle, float newRate, float dt);

    // return the filtered angle
    float getAngle();

    // Return the biased input rate
    float getRate();

    void setQangle(float Q_angle);

    // tune high tight the filter follows the input values
    // Default is 0.003
    // Raise this to follow input more closely,
    // lower this to smooth result
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();
private:
    float Q_angle; 		// Process noise variance for the accelerometer
    float Q_bias; 		// Process noise variance for the gyro bias
    float R_measure; 	// Measurement noise variance - this is the variance of the measurement noise

    float angle; 		// The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; 		// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; 		// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P00,P01,P10,P11; 	// Error covariance matrix - This is a 2x2 matrix
};

#endif /* KALMAN_KALMAN_H_ */
