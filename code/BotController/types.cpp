
#include <Arduino.h>
#include <types.h>
#include <setup.h>
#include <Util.h>

BotMovement::BotMovement() {
	reset();
}

void BotMovement::print() {
	logging("x=");
	x.print();
	logging(" y=");
	y.print();
}

BotMovement::BotMovement(const State &x, const State& y, float omega) {
	this->x = x;
	this->y = y;
	this->omega = omega;
}

BotMovement::BotMovement(const BotMovement& t) {
	x = t.x;
	y = t.y;
	omega = t.omega;
}

BotMovement& BotMovement::operator=(const BotMovement& t) {
	x = t.x;
	y = t.y;
	omega = t.omega;
	return *this;
}

void BotMovement::reset() {
	x.reset();
	y.reset();
	omega = 0;
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
	increaseWithDifferentiableAcceleration(x.speed, x.accel, target.x.speed, dT);
	increaseWithDifferentiableAcceleration(y.speed, y.accel, target.y.speed, dT);

	omega = constrain(target.omega, omega - MaxBotOmegaAccel*dT, omega +  MaxBotOmegaAccel*dT);
	omega = constrain(omega, -MaxBotOmega, +MaxBotOmega);
}



Point::Point() {
}

void Point::print() {
	logging("(");
	logging(x,2,3);
	logging(",");
	logging(y,2,3);
	logging(")");

}

Point::Point(float x, float y) {
	this->x= x;
	this->y= y;
}

Point::Point(const Point& t) {
	x = t.x;
	y = t.y;
}

Point& Point::operator=(const Point& t) {
	x = t.x;
	y = t.y;
	return *this;
}



Speed::Speed() {
}

void Speed::print() {
	logging("(");
	logging(x,2,3);
	logging(",");
	logging(y,2,3);
	logging(")");

}

Speed::Speed(float x, float y) {
	this->x= x;
	this->y= y;
}

Speed::Speed(const Speed& t) {
	x = t.x;
	y = t.y;
}

Speed& Speed::operator=(const Speed& t) {
	x = t.x;
	y = t.y;
	return *this;
}



Pose::Pose() {
}

void Pose::print() {
	logging("(");
	logging(p.x,2,3);
	logging(",");
	logging(p.y,2,3);
	logging(",");
	logging(orientation,2,3);

	logging(")");

}

Pose::Pose(float x, float y, float orientation) {
	this->p.x= x;
	this->p.y= y;
	this->orientation= orientation;

}


Pose::Pose(const Point & p, float orientation) {
	this->p = p;
	this->orientation= orientation;
}


Pose::Pose(const Pose& t) {
	p = t.p;
	orientation = t.orientation;
}

Pose& Pose::operator=(const Pose& t) {
	p = t.p;
	orientation = t.orientation;
	return *this;
}



Speed3D::Speed3D() {
}

void Speed3D::print() {
	logging("(");
	logging(p.x,3,1);
	logging(",");
	logging(p.y,3,1);
	logging(",");
	logging(orientation,3,1);

	logging(")");

}

Speed3D::Speed3D(float x, float y, float orientation) {
	this->p.x= x;
	this->p.y= y;
	this->orientation= orientation;

}


Speed3D::Speed3D(const Speed & p, float orientation) {
	this->p = p;
	this->orientation= orientation;
}


Speed3D::Speed3D(const Speed3D& t) {
	p = t.p;
	orientation = t.orientation;
}

Speed3D& Speed3D::operator=(const Speed3D& t) {
	p = t.p;
	orientation = t.orientation;
	return *this;
}


State::State() {
	reset();
}

void State::print() {
	logging("(");
	logging(pos,2,3);
	logging(",");
	logging(speed,2,3);
	logging(",");
	logging(accel,2,3);
	logging(")");

}

State::State(float pos, float speed, float accel) {
	this->pos = pos;
	this->speed = speed;
	this->accel = accel;
}

State::State(const State& t) {
	pos = t.pos;
	speed = t.speed;
	accel = t.accel;
}

State& State::operator=(const State& t) {
	pos = t.pos;
	speed = t.speed;
	accel = t.accel;
	return *this;
}

void State::reset() {
	pos = 0;
	speed = 0;
	accel = 0;
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
