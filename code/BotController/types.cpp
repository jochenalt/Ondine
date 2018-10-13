
#include <Arduino.h>
#include <types.h>
#include <setup.h>

BotMovement::BotMovement() {
		speedX = 0;
		speedY = 0;
		accelX = 0;
		accelY = 0;
		omega = 0;
}

BotMovement::BotMovement(float speedX, float speedY, float omega) {
		this->speedX = speedX;
		this->speedY = speedY;
		this->accelX = accelX;
		this->accelY = accelY;

		this->omega = omega;
}

BotMovement::BotMovement(const BotMovement& t) {
		speedX = t.speedX;
		speedY = t.speedY;
		accelX = t.accelX;
		accelY = t.accelY;
		omega = t.omega;
}

BotMovement& BotMovement::operator=(const BotMovement& t) {
		speedX = t.speedX;
		speedY = t.speedY;
		accelX = t.accelX;
		accelY = t.accelY;
		omega = t.omega;
		return *this;
}

// increase currentSpeed towards newSpeed, adapt currentAcceleration.
// Consider maximum acceleration MaxBotAccel, and
// use a trapezoid speed profile with round corners (jerkless acceleration)
void increaseWithDifferentiableAcceleration(float &currentSpeed, float &currentAccel, float newSpeed,float dT) {
	if (dT > OneMicrosecond_s) {
		// an acceleration corresponds with the tilt angle of the bot.
		// Since this can not change immediately, the acceleration needs to be ramped up

		// compute max speed we can reach with max acceleration
		float targetSpeed = constrain(newSpeed, currentSpeed - MaxBotAccel*dT, currentSpeed +  MaxBotAccel*dT);
		targetSpeed = constrain(targetSpeed, -MaxBotSpeed, +MaxBotSpeed);

		// what acceleration would be required to get to that speed?
		float targetAccel = (targetSpeed - currentSpeed)/dT;

		// limit changes in that acceleration to make the movement jerkless
		currentAccel = constrain(targetAccel, currentAccel - MaxBotAccelAccel*dT, currentAccel + MaxBotAccelAccel*dT);

		// compute the speed out of the jerkless acceleration
		currentSpeed += currentAccel*dT;
	}
	else {
		currentSpeed = 0;
		currentAccel = 0;
	}
}

void BotMovement::rampUp(const BotMovement& target, float dT) {
	increaseWithDifferentiableAcceleration(speedX, accelX, target.speedX, dT);
	increaseWithDifferentiableAcceleration(speedY, accelY, target.speedY, dT);

	omega = constrain(target.omega, omega - MaxBotOmegaAccel*dT, omega +  MaxBotOmegaAccel*dT);
	omega = constrain(omega, -MaxBotOmega, +MaxBotOmega);
}


// return Rz * Ry * Rz
void computeZYXRotationMatrix(float eulerX, float eulerY, float eulerZ, matrix33_t& m) {
	float  sinX = sin(eulerX);
	float  cosX = cos(eulerX);
	float  sinY = sin(eulerY);
	float  cosY = cos(eulerY);
	float  sinZ = sin(eulerZ);
	float  cosZ = cos(eulerZ);

	ASSIGN(m[0], cosZ*cosY, 	-sinZ*cosX+cosZ*sinY*sinX,  	sinZ*sinX+cosZ*sinY*cosX);
	ASSIGN(m[1], sinZ*cosY, 	cosZ*cosX + sinZ*sinY*sinX, 	cosZ*sinX+sinZ*sinY*cosX);
	ASSIGN(m[2], -sinY,	 		cosY*sinX,						cosY*cosX);
}

void computeInverseMatrix(matrix33_t m, matrix33_t &inverse) {

	float det_denominator =
				     ((m[0][0]) * m[1][1] * m[2][2]) +
			         ((m[0][1]) * m[1][2] * m[2][0]) +
		             ((m[0][2]) * m[1][0] * m[2][1]) -
		             ((m[2][0]) * m[1][1] * m[0][2]) -
		             ((m[2][1]) * m[1][2] * m[0][0]) -
		             ((m[2][2]) * m[1][0] * m[0][1]);

	float detRezi = 1.0 / det_denominator;
	ASSIGN(inverse[0],
		detRezi*(((m[1][1]) * m[2][2] - (m[1][2]) * m[2][1])),
		detRezi*(((m[0][2]) * m[2][1] - (m[0][1]) * m[2][2])),
		detRezi*(((m[0][1]) * m[1][2] - (m[0][2]) * m[1][1])));
	ASSIGN(inverse[1],
		detRezi*(((m[1][2]) * m[2][0] - (m[1][0]) * m[2][2])),
		detRezi*(((m[0][0]) * m[2][2] - (m[0][2]) * m[2][0])),
		detRezi*(((m[0][2]) * m[1][0] - (m[0][0]) * m[1][2])));
	ASSIGN(inverse[2],
		detRezi*(((m[1][0]) * m[2][1] - (m[1][1]) * m[2][0])),
		detRezi*(((m[0][1]) * m[2][0] - (m[0][0]) * m[2][1])),
		detRezi*(((m[0][0]) * m[1][1] - (m[0][1]) * m[1][0])));

}

const float floatPrecision = 0.00000001;
void computeEuler(matrix33_t m, float eulerX, float eulerY, float eulerZ) {
	float beta = atan2(-m[2][0], sqrt(m[0][0]*m[0][0] + m[1][0]*m[1][0]));
	float gamma = 0;
	float alpha = 0;
	if (abs(beta-HALF_PI) < floatPrecision) {
		alpha = 0;
		gamma = atan2(m[0][1], m[1][1]);
	} else {
			if (abs(beta + HALF_PI) < floatPrecision) {
				alpha = 0;
				gamma = -atan2f(m[0][1], m[1][1]);
			} else {
				alpha = atan2f(m[1][0],m[0][0]);
				gamma = atan2f(m[2][1], m[2][2]);
			}
	}
	eulerX = alpha;
	eulerY = beta;
	eulerZ = gamma;
}

// result = v * m
void vectorTimesMatrix(vector3 v, matrix33_t m, vector3 &result) {
	result[0] = v[0]*m[0][0] + v[1]*m[0][1]  + v[2]*m[0][2];
	result[1] = v[0]*m[1][0] + v[1]*m[1][1]  + v[2]*m[1][2];
	result[2] = v[0]*m[2][0] + v[1]*m[2][1]  + v[2]*m[2][2];
}

void multiplyMatrix(matrix33_t v, matrix33_t m, matrix33_t &result) {
	 for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        	result[i][j]=0;

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            for(int k = 0; k < 3; ++k)
                result[i][j] += v[i][k] * m[k][j];
}
