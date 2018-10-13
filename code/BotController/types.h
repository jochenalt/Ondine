
#ifndef __TYPES_H_
#define __TYPES_H_

// used as rotation matrix
typedef float matrix33_t[3][3];

typedef float vector3[3];

// meaning of a vector's item
enum Dimension { X=0,Y=1,Z=2 };

// returns Rz * Ry * Rx
void computeZYXRotationMatrix(float eulerX, float eulerY, float eulerZ, matrix33_t &m);

// inverse = m^(-1)
void computeInverseMatrix(matrix33_t m, matrix33_t &inverse);

// result = v * m
void vectorTimesMatrix(vector3 v,matrix33_t m, vector3 &result);

// result = v*m
void multiplyMatrix(matrix33_t v, matrix33_t m, matrix33_t &result);

void computeEuler(matrix33_t m, float eulerX, float eulerY, float eulerZ);

// convenience macro: set a 3-vector
#define ASSIGN(m,a,b,c) m[0] = (a);m[1] = (b); m[2] = (c)

class BotMovement {
public:
	BotMovement();
	BotMovement(float speedX, float speedY, float omega);
	BotMovement(const BotMovement& t);
	BotMovement& operator=(const BotMovement& t);

	// increase movement towards target in a jerkless manner,
	// i.e. the acceleration used increases with a constant rate
	void rampUp(const BotMovement& target, float dT);

	float speedX = 0;
	float speedY = 0;
	float accelX = 0;
	float accelY = 0;
	float omega = 0;
};



#endif
