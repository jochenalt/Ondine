/*
 * Kinematics.cpp
 *
 * Created: 30.11.2012 16:23:25
 * Author: JochenAlt
 */ 


#include "Arduino.h"
#include "Kinematics.h"

Kinematix kin;

const float WheelRadius = 35.;
const float BallRadius = 90.;
const float WheelAngleRad= radians(45.);
const float MaxWheelSpeed = 2000.;

// convenience macro: set a 3-vector
#define VECTOR_SET(m,a,b,c) m[0] = (a);m[1] = (b); m[2] = (c)

// return the construction matrix (CM) and compute its inverse matrix used for inverse kinematics
// for conveniency use floats, since this matrix is computed during startup only
void Kinematix::setupConstructionMatrix() {
		float a = -1.0/WheelRadius;
		float cos_phi = cos(WheelAngleRad);
		float sin_phi = sin(WheelAngleRad);
		

		// define the construction matrix
		VECTOR_SET(cm[0],                      0, a*cos_phi,          -a*sin_phi);
		VECTOR_SET(cm[1], -a*sqrt(3.)/2.*cos_phi, -a*cos_phi/2.,      -a*sin_phi);
		VECTOR_SET(cm[2],  a*sqrt(3.)/2.*cos_phi, -a*cos_phi/2.,      -a*sin_phi);

		// compute inverse of construction matrix
		float det_denominator = 
					     ((cm[0][0])*cm[1][1] * cm[2][2]) +
 			             ((cm[0][1])*cm[1][2] * cm[2][0]) +
			             ((cm[0][2])*cm[1][0] * cm[2][1]) -
			             ((cm[2][0])*cm[1][1] * cm[0][2]) -
			             ((cm[2][1])*cm[1][2] * cm[0][0]) -
			             ((cm[2][2])*cm[1][0] * cm[0][1]);

		float detRezi = 1.0 / det_denominator;
		VECTOR_SET(icm[0],
			detRezi*(((cm[1][1]) * cm[2][2] - (cm[1][2]) * cm[2][1])),
			detRezi*(((cm[0][2]) * cm[2][1] - (cm[0][1]) * cm[2][2])),
			detRezi*(((cm[0][1]) * cm[1][2] - (cm[0][2]) * cm[1][1])));
		VECTOR_SET(icm[1],
			detRezi*(((cm[1][2]) * cm[2][0] - (cm[1][0]) * cm[2][2])),
			detRezi*(((cm[0][0]) * cm[2][2] - (cm[0][2]) * cm[2][0])),
			detRezi*(((cm[0][2]) * cm[1][0] - (cm[0][0]) * cm[1][2])));
		VECTOR_SET(icm[2],
			detRezi*(((cm[1][0]) * cm[2][1] - (cm[1][1]) * cm[2][0])),
			detRezi*(((cm[0][1]) * cm[2][0] - (cm[0][0]) * cm[2][1])),
			detRezi*(((cm[0][0]) * cm[1][1] - (cm[0][1]) * cm[1][0])));
}



// tiltRotationMatrix (TRM) is the rotation matrix that is able to compensate the position where
// the ball touches the ground. This points moves if the robot tilts, so when doing forward and inverse kinematics
// this angle needs to be taken into account when the wheel speed is computed out of x,y, omega
void Kinematix::computeTiltRotationMatrix(float pTiltX, float pTiltY) {
	
	// do it only if tilt angles have really changed
	// This is important, since forward and inverse kinematics per loop have the same angles, this doubles performance
	static bool alreadyComputed = false; // compute at least the first time, even if angles are all zero
	static float lastTiltX, lastTiltY;
	
	if  (alreadyComputed && 
		(lastTiltX == pTiltX) &&
		(lastTiltY == pTiltY))
		return;
	
	alreadyComputed = true;
	lastTiltX = pTiltX;
	lastTiltY = pTiltY;
	
	// compute sin and cos, needed in rotation matrix
	float sinX = sin(pTiltY);
	float cosX = cos(pTiltY);
	float sinY = sin(pTiltX);
	float cosY = cos(pTiltX);

	// compute Tilt Rotation Matrix (TRM). All values are between -1..1, so use FP16
	// fixed point arithmetics. accuracy of TRM is better than 1%
	// computation is coming from kinematix.xls
	VECTOR_SET(trm[0],       cosY,     0,      sinY);
	VECTOR_SET(trm[1],  sinX*sinY,  cosX,-sinX*cosY);
	VECTOR_SET(trm[2], -cosX*sinY,  sinX, cosX*cosY);
}

// compute speed of all motors depending from the speed in the IMU's coordinate system in (Vx, Vy, OmegaZ) 
// corrected by the tilt of the imu pTiltX, pTiltY 
void Kinematix::computeWheelSpeed( float pVx, float pVy, float pOmegaZ,
		float pTiltX, float pTiltY,
		float pWheel_speed[3]) {
	
	// this matrix depends on the tilt angle and corrects the kinematics 
	// due to the slightly moved touch point of the ball
	kin.computeTiltRotationMatrix(pTiltX,pTiltY);

	// rotate construction matrix by tilt (by multiplying with tilt rotation matrix)
	// compute only those fields that are required afterwards (so we need only 10 multiplications instead 9* (3*3) of a regular of matrix multiplcation)
	float m01_11=  cm[0][1]*trm[1][1];
	float m01_21 = cm[0][1]*trm[2][1];
	float m10_00 = cm[1][0]*trm[0][0];
	float m10_10 = cm[1][0]*trm[1][0];
	float m10_20 = cm[1][0]*trm[2][0];
	float m11_11 = cm[1][1]*trm[1][1];
	float m11_21 = cm[1][1]*trm[2][1];
	float m02_02 = cm[0][2]*trm[0][2];
	float m02_22 = cm[0][2]*trm[2][2];
	float m02_12 = cm[0][2]*trm[1][2];

	// convert to radian
	float  lBotZSpeed = -pOmegaZ * BallRadius;

	// final computation of kinematics:
	// compute wheel's speed in °/s by (wheel0,wheel1,wheel2) = Construction-Matrix * Tilt-Compensation Matrix * (Vx, Vy, Omega)
	float wheelSpeed0 = ((m01_11 + m02_12)*pVx	         - m02_02*pVy             + (m01_21 + m02_22)*lBotZSpeed)           ;
	float wheelSpeed1 = ((m10_10 + m11_11 + m02_12)*pVx  - (m10_00 + m02_02)*pVy  + (m10_20 + m11_21 + m02_22)*lBotZSpeed ) ;
	float wheelSpeed2 = ((-m10_10+ m11_11 + m02_12)*pVx  + (m10_00 - m02_02)*pVy  + (-m10_20 + m11_21 + m02_22)*lBotZSpeed) ;

	// alarm: if one wheel is supposed to run faster than it can
	// reduce the speed of all wheels in proportion, so that the 
	// fastest wheel has max speed
	float absWheelSpeed0 = abs(wheelSpeed0);
	float absWheelSpeed1 = abs(wheelSpeed1);
	float absWheelSpeed2 = abs(wheelSpeed2);

	// while one wheel's to-be speed exceeds max speed
	// compute reduction factor for fastest wheel and reduce 
	// speed of all wheels accordingly
	while ((absWheelSpeed0 > MaxWheelSpeed) ||
		   (absWheelSpeed1 > MaxWheelSpeed) ||
		   (absWheelSpeed2 > MaxWheelSpeed)) {

		float factor;
		if (absWheelSpeed0 > MaxWheelSpeed)
			factor = MaxWheelSpeed / absWheelSpeed0;
		else
		if (absWheelSpeed1 > MaxWheelSpeed)
			factor = MaxWheelSpeed / absWheelSpeed1;
		else
			factor = MaxWheelSpeed / absWheelSpeed2;
		wheelSpeed0 = wheelSpeed0 * factor;
		wheelSpeed1 = wheelSpeed1 * factor;
		wheelSpeed2 = wheelSpeed2 * factor;
		
		absWheelSpeed0 = abs(wheelSpeed0);
		absWheelSpeed1 = abs(wheelSpeed1);
		absWheelSpeed2 = abs(wheelSpeed2);
	}

	pWheel_speed[0] = wheelSpeed0;
	pWheel_speed[1] = wheelSpeed1;
	pWheel_speed[2] = wheelSpeed2;
}

// compute actual speed in the coord-system of the IMU out of the encoder's data depending on the given tilt
void Kinematix::computeActualSpeed( float pWheel[3],
									float pTiltX, float pTiltY,
									float& pVx, float& pVy, float& pOmega) {
	// this matrix depends on the tilt angle and corrects the kinematics 
	// due to the moved touch point of the ball
	kin.computeTiltRotationMatrix(pTiltX,pTiltY);

	// compute the sparse result of the construction matrix * tilt compensation matrix
	// (multiply only those fields that are required afterwards, so we have only 10 instead of 81 multiplications)
	int16_t m00_01 = trm[0][0] * icm[0][1];
	int16_t m02_20 = trm[0][2] * icm[2][0];
	int16_t m10_01 = trm[1][0] * icm[0][1];
	int16_t m11_10 = trm[1][1] * icm[1][0];
	int16_t m11_11 = trm[1][1] * icm[1][1];
	int16_t m12_20 = trm[1][2] * icm[2][0];
	int16_t m20_01 = trm[2][0] * icm[0][1];
	int16_t m21_10 = trm[2][1] * icm[1][0];
	int16_t m21_11 = trm[2][1] * icm[1][1];
	int16_t m22_20 = trm[2][2] * icm[2][0];
							
	// compute inverse kinematics matrix					
	pVx    =     ( m11_10 + m12_20)              * pWheel[0]
			      + ( m10_01 + m11_11 + m12_20)  * pWheel[1]
				  + (-m10_01 + m11_11 + m12_20)  * pWheel[2];
	pVy    =     (-m02_20)                       * pWheel[0]
  				  + (-m00_01 - m02_20)           * pWheel[1]
		          + ( m00_01 - m02_20)           * pWheel[2];
	pOmega = (   (-m21_10 - m22_20)              * pWheel[0]
				  + (-m20_01 - m21_11 - m22_20)  * pWheel[1]
				  + ( m20_01 - m21_11 - m22_20)  * pWheel[2]) / BallRadius;
}


void Kinematix::setup() {
	// create construction matrix and its inverse
	setupConstructionMatrix();
}

void Kinematix::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 't':
		testInverseKinematics();
		testPerformanceKinematics();
		testKinematics();
		break;
	case 'h':
		printHelp();
		break;
	case 27:
		deactivateMenu();
		return;
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		Serial1.println(">");
	}
}


void Kinematix::testKinematics() {
	
	setupConstructionMatrix();
	float cm00 = cm[0][0];
	float cm01 = cm[0][1];
	float cm02 = cm[0][2];
	float cm10 = cm[1][0];
	float cm11 = cm[1][1];
	float cm12 = cm[1][2];
	float cm20 = cm[2][0];
	float cm21 = cm[2][1];
	float cm22 = cm[2][2];
	
	Serial1.println(F("construction matrix"));
	Serial1.print(cm00);
	Serial1.print(cm01);
	Serial1.print(cm02);Serial1.println();
	Serial1.print(cm10);
	Serial1.print(cm11);
	Serial1.print(cm12);Serial1.println();
	Serial1.print(cm20);
	Serial1.print(cm21);
	Serial1.print(cm22);Serial1.println(" ");

		
	float lVx,lVy,lOmega;
	lVx = 0;
	lVy = 0;

	lOmega=35.0;
	Serial1.print(F("Vx="));
	
	Serial1.print(lVx);
	Serial1.print(F(" Vy="));
	Serial1.print(lVy);
	Serial1.print(F(" Omega="));
	Serial1.print(lOmega);
	Serial1.println();

	float lTiltX,lTiltY;
	lTiltX = 0;
	lTiltY = 0;

	Serial1.print(F("TiltX="));
	Serial1.print(lTiltX);
	Serial1.print(F(" TiltY="));
	Serial1.print(lTiltY);
	Serial1.println();
	
	float pWheel_speed[3] = {0,0,0};
	 computeWheelSpeed( lVx, lVy, lOmega,
	 					lTiltX, lTiltY,
						pWheel_speed);
	float lWheel1 = pWheel_speed[0];
	float lWheel2 = pWheel_speed[1];
	float lWheel3 = pWheel_speed[2];
	
	Serial1.print(F("W1="));
	Serial1.print(lWheel1);
	Serial1.print(F(" W2="));
	Serial1.print(lWheel2);
	Serial1.print(F(" W3="));
	Serial1.print(lWheel3);
	Serial1.println();
}	

void Kinematix::testInverseKinematics() {
	
	setupConstructionMatrix();
	float icm00 = icm[0][0];
	float icm01 = icm[0][1];
	float icm02 = icm[0][2];
	float icm10 = icm[1][0];
	float icm11 = icm[1][1];
	float icm12 = icm[1][2];
	float icm20 = icm[2][0];
	float icm21 = icm[2][1];
	float icm22 = icm[2][2];

	Serial1.println(F("inverse construction matrix"));
	Serial1.print(icm00);
	Serial1.print(icm01);
	Serial1.print(icm02);Serial1.println(" ");
	Serial1.print(icm10);
	Serial1.print(icm11);
	Serial1.print(icm12);Serial1.println(" ");
	Serial1.print(icm20);
	Serial1.print(icm21);
	Serial1.print(icm22);Serial1.println(" ");

	// speed of wheels in °/s
	float lWheel1 = -758.9;
	float lWheel2 = 36.4;
	float lWheel3 = -133.7;
	
	Serial1.print(F("W1="));
	Serial1.print(lWheel1);
	Serial1.print(F(" W2="));
	Serial1.print(lWheel2);
	Serial1.print(F(" W3="));
	Serial1.print(lWheel3);
	Serial1.println();
						
	float  lTiltX,lTiltY;
	lTiltX = 20.0;
	lTiltY = -15;
	Serial1.print(F("TiltX="));
	Serial1.print(lTiltX);
	Serial1.print(F(" TiltY="));
	Serial1.print(lTiltY);Serial1.println();
	
	// this matrix depends on the tilt angle and corrects the 
	computeTiltRotationMatrix(lTiltX,lTiltY);

	
	float lVx = 0;
	float lVy = 0;
	float lOmega = 0;
	
	float wheel[3] = {0,0,0};
	wheel[0] = lWheel1;
	wheel[1] = lWheel2;
	wheel[2] = lWheel3;
	
	computeActualSpeed( wheel,
				lVx, lVy, lTiltX, lTiltY,lOmega);
				

	Serial1.print(F("Vx="));
	Serial1.print(lVx);
	Serial1.print(F(" Vy="));
	Serial1.print(lVy);
	Serial1.print(F(" Omega="));
	Serial1.print(lOmega);
	Serial1.println();
}


void Kinematix::testPerformanceKinematics() {
	
	Serial1.println(F("Kinematics performance"));
	unsigned long start =	millis();
	unsigned long end =	millis();
	Serial1.print("End ms=");
	Serial1.println(end-start);
	
	int i = 0;
	Serial1.println(F("Start"));
	start =	millis();
	for (i = 0;i<1000;i++) {
		float lVx,lVy,lOmega;
		lVx = 300;
		lVy = -100;
		lOmega=35;

		float lTiltX = 20;
		float lTiltY = 15;
		float pWheel_speed[3] = {0,0,0};
		computeWheelSpeed( lVx, lVy, lOmega,
	 					lTiltX, lTiltY,
						pWheel_speed);
						
		computeActualSpeed(pWheel_speed,0,0,
					lVx, lVy, lOmega);
	}
	end = millis();
	Serial1.println(F("Stop"));

	Serial1.print((end-start),DEC);
	Serial1.print("ms for ");
	Serial1.print(i,DEC);
	Serial1.print(" loops, ");
	Serial1.print(float((end-start)) / float(i));
	Serial1.println("ms");
}

void Kinematix::testTRM() {
	for (int j = 1;j<20;j=j+5) {
		float error  = 0;
		for (float i = 0.0;i<2*PI;i=i+0.1) {
			float x,y;
			x = sin(float(i))*j;
			y = cos(float(i)) *j;


			kin.computeTiltRotationMatrix(x,y);

			float sin_tilt_x = sin(radians(x));
			float cos_tilt_x = cos(radians(x));
			float sin_tilt_y = sin(radians(y));
			float cos_tilt_y = cos(radians(y));

			error += (abs(sin_tilt_x)==0)?0: abs ((trm[0][0] - sin_tilt_x) / sin_tilt_x);
			error += (sin_tilt_y==0)?0:abs (((trm[0][2]) - sin_tilt_y) / sin_tilt_y);

			error += (sin_tilt_x*sin_tilt_y == 0)?0:abs (((trm[1][0]) - sin_tilt_x*sin_tilt_y) /(sin_tilt_x*sin_tilt_y));
			error += (cos_tilt_x==0)?0:abs ((((trm[1][1]),14) - cos_tilt_x) / cos_tilt_x);
			error += (sin_tilt_x*cos_tilt_y==0)?0:abs ((((trm[1][2]),14) - (-sin_tilt_x*cos_tilt_y)) / (sin_tilt_x*cos_tilt_y));

			error += (cos_tilt_x*sin_tilt_y==0)?0:abs ((((trm[2][0]),14) + cos_tilt_x*sin_tilt_y) / (cos_tilt_x*sin_tilt_y));
			error += (sin_tilt_x==0)?0:abs ((((trm[2][1]),14) - sin_tilt_x) / sin_tilt_x);
			error += (cos_tilt_x*cos_tilt_y==0)?0:abs ((((trm[2][2]),14) - cos_tilt_x*cos_tilt_y) / (cos_tilt_x*cos_tilt_y));

			Serial1.print(int(error),DEC);
		}	
		error = 0;
	}	
}

void Kinematix::printHelp() {
	Serial1.println(F("Kinematics"));
	Serial1.println(F("t - test kinematics"));
	Serial1.println(F("h   - help"));
	Serial1.println(F("0   - exit"));
}
