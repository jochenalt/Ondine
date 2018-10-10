/*
 * Kinematics.h
 *
 * Created: 05.12.2012 01:12:11
 * Author: JochenAlt
 *
 * Class providing the kinematics, i.e. it computes the wheelspeed out of the x,y,omega speed and vice versa
 * - it knows the construction of the robot and all the data like ball radius, wheel radius
 * - setup computes the several rotation matrixes and prepares them so that they can be computed efficiently during
 *   main loop
 *
 * The computations are explained in the attached excel Kinematics.xls
 */ 

#ifndef KINEMATIX_H_
#define KINEMATIX_H_

#include "MenuController.h"
#include "types.h"


class Kinematix {
	public:
		Kinematix () {}

		// setup construction matrix, i.e. that precomputed matrix defining the construction specifics
		void setup();
		
		// forward kinematix: speed x/y/omega -> wheel speed
		void computeWheelSpeed( float pVx, float pVy, float pOmegaZ,
				float pTiltX, float pTiltY,
				float pWheel1_speed[3]);

		// inverse kinematix: wheel speed -> speed xy/omega
		void computeActualSpeed(float pWheel[3],
						float  pTiltX, float  pTiltY,
						float & pVx, float & pVy, float & pOmega);

		// test kinematics, to be used in debugger
		void testKinematics();
		// test inverse kinematics, to be used in debugger
		void testInverseKinematics();
		// check performance
		void testPerformanceKinematics();
		
		// TRM is tiltRotationMatrix, the rotation matrix that is able to compensate the position where
		// the ball touches the ground. This points moves if the robot tilts.
		// Function to test this in the debugger.
		void testTRM();

	private:
		// compute the tilt rotation matrix, used in kinematics and inverse kinematics
		void computeTiltRotationMatrix(float pTiltX, float  pTiltY);
		// pre-compute kinematics so that during the loop just a couple of multiplications are required.
		void setupConstructionMatrix();
		
		matrix33_t cm;	    	// construction matrix, computed once during startup
		matrix33_t icm;	 	// inverse construction matrix, computed once during startup
		matrix33_t trm;     	// tilt rotation matrix to correct speed/omega depending on the tilt angle, computed in each loop

		bool titleCompensationMatrixComputed = false;
		float lastTiltX = 0;
		float lastTiltY = 0;

};



#endif /* KINEMATIX_H_  */
