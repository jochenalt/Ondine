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

#include <MenuController.h>
#include <types.h>
#include <IMU.h>

class Trajectory : public Menuable {
public:
	Trajectory() {};
	virtual ~Trajectory() {};

	void setup(MenuController* menuCtrl);
	void loop();

	void setSpeed(const Speed3D& speed);
	void setOdom(const Pose& odom);

	BotMovement getCurrentBotMovement();

	// ascii menu, menu commands are implemented there
	virtual void menuLoop(char ch, bool continously);

	// print ascii help to console
	virtual void printHelp();

private:
	enum MovementMode { SPEED, POSITION };

	Pose targetPose;

	Speed3D targetSpeed;

	BotMovement current;
};

#endif /* BALLDRIVE_H_ */
