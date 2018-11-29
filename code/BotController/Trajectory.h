/*
 * BallEngine.h
 *
 * Takes three single wheels, adds kinematics and provides methods to
 * set and retrieve speed in terms of (x,y,omega).
 *
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <libraries/MenuController.h>
#include <types.h>
#include <setup.h>

class Trajectory : public Menuable {
public:
	Trajectory() {};
	virtual ~Trajectory() {};

	void setup(MenuController* menuCtrl);
	void loop();

	void setSpeed(const Speed3D& speed);
	void setOdom(const Pose& pose, uint32_t approachingTime, const Speed& targetSpeed);
	void setAcceleration(float accel = MaxBotAccel);

	BotMovement getCurrentBotMovement();

	// ascii menu, menu commands are implemented there
	virtual void menuLoop(char ch, bool continously);

	// print ascii help to console
	virtual void printHelp();

private:
	enum MovementMode { SPEED, POSITION };

	Pose targetPose;
	Speed targetSpeed;
	uint32_t targetTime = 0;;
	BotMovement current;

	uint32_t lastLoopTime = 0;
};

#endif /* BALLDRIVE_H_ */
