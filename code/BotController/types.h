
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



class Point {
public:
	Point();
	Point(float x, float y);
	Point(const Point& t);
	Point& operator=(const Point& t);
	void print();

	float x = 0;
	float y = 0;
};


class Speed {
public:
	Speed();
	Speed(float x, float y);
	Speed(const Speed& t);
	Speed& operator=(const Speed& t);
	void print();

	float x = 0;
	float y = 0;
};

class Pose {
public:
	Pose();
	Pose(float x, float y, float orientation);
	Pose(const Point& p, float orientation);

	Pose(const Pose& t);
	Pose& operator=(const Pose& t);
	void print();

	Point p;
	float orientation; // [rad];
};


class Speed3D {
public:
	Speed3D();
	Speed3D(float x, float y, float orientation);
	Speed3D(const Speed& p, float orientation);

	Speed3D(const Speed3D& t);
	Speed3D& operator=(const Speed3D& t);
	void print();

	Speed p;
	float orientation; // [rad/s];
};

class State {
public:
	State();
	State(float pos, float speed, float accel);
	State(const State& t);
	State& operator=(const State& t);
	void reset();
	void print();

	float pos = 0;	 	// [m]
	float speed = 0;	// [m/s]
	float accel = 0;	// [m/s*s]
};



class BotMovement {
public:
	BotMovement();
	BotMovement(const State &x, const State& y, float omega);

	BotMovement(const BotMovement& t);
	BotMovement& operator=(const BotMovement& t);
	void reset();
	void print();

	// increase movement towards target in a jerkless manner,
	// i.e. the acceleration used increases with a constant rate
	void rampUp(const BotMovement& target, float dT);

	State x;
	State y;
	float omega = 0;
};



#endif
