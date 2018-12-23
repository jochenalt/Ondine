/*
 * BallEngine.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <BallDrive.h>
#include <types.h>

void BallDrive::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);
	engine.setup(menuCtrl);
	kinematics.setup();
	reset();
	lastCall_us = 0;
}


void BallDrive::printHelp() {
	logger->println();
	logger->println("Ball Drive");
	logger->println();
	logger->println("p - power");
	logger->println("e - enable");
	logger->println("q/a - modify x speed");
	logger->println("w/s - modify y speed");
	logger->println("y/x - modify omega speed");
	logger->println("o/l - modify tilt x");
	logger->println("i/k - modify tilt y");
	logger->println("t   - test kinmatics");

	logger->println("b - single wheel control");

	logger->println("0 - nullify speed");

	logger->println("ESC");
}

void BallDrive::setSpeed(float speedX, float speedY, float omega,
		 	 	 	 	 float angleX, float angleY) {

	// apply kinematics to compute wheel speed out of x,y, omega
	float  newWheelSpeed[3] = { 0, 0, 0}; // [rev/s]
	kinematics.computeWheelSpeed(speedX, speedY, omega,
 							angleX,angleY,
							newWheelSpeed);

	engine.setWheelSpeed(newWheelSpeed);
}

// compute the current speed since the last invocation.
// returns 0 when called first (assuming the we start without motion)
void BallDrive::getSpeed(uint32_t now_us, const IMUSample &sample, BotMovement &current) {

	if ((now_us > lastCall_us) && (lastCall_us > 0))  {
		float dT = ((float)(now_us - lastCall_us))/1000000.0; // [s]
		lastCall_us = now_us;

		// fetch motor encoder values to compute real wheel position
		float angleChange[3] = {0,0,0};
		engine.getWheelAngleChange(angleChange);

		float currentWheelSpeed[3]; // [rev/s]
		currentWheelSpeed[0] = angleChange[0]  / (TWO_PI * dT);
		currentWheelSpeed[1] = angleChange[1]  / (TWO_PI * dT);
		currentWheelSpeed[2] = angleChange[2]  / (TWO_PI * dT);

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		kinematics.computeActualSpeed(  currentWheelSpeed,
										sample.plane[X].angle, sample.plane[Y].angle,
										current.x.speed, current.y.speed, current.omega);

		current.x.pos += dT*current.x.speed;
		current.y.pos += dT*current.y.speed;

		current.x.accel = (current.x.speed - lastSpeedX)/dT;
		current.y.accel = (current.y.speed - lastSpeedY)/dT;

		lastSpeedX = current.x.speed;
		lastSpeedY = current.y.speed;

		// engine.resetAngle();

	} else {
		lastSpeedX = current.x.speed;
		lastSpeedY = current.y.speed;
		lastCall_us = now_us;
	}
}

void BallDrive::loop(uint32_t now_us) {
	// drive the motors
	// due to use of brushless motors, this requires permanent invokation of loop
	engine.loop(now_us);

}

void BallDrive::menuLoop(char ch, bool continously) {
	bool cmd = true;
	switch (ch) {
	case '0':
		menuSpeedX = 0;
		menuSpeedY = 0;
		menuOmega = 0;
		menuAngleX = 0;
		menuAngleY = 0;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'q':
		menuSpeedX += 0.001;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'a':
		menuSpeedX -= 0.001;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'w':
		menuSpeedY += 0.001;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 's':
		menuSpeedY -= (0.001);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'y':
		menuOmega += radians(0.1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'x':
		menuOmega -= radians(0.1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'o':
		menuAngleX += radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'l':
		menuAngleX -= radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'i':
		menuAngleY += radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'k':
		menuAngleY -= radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'e':
		enable(!isEnabled());
		break;
	case 'b':
		engine.pushMenu();
		break;
	case 'h':
		printHelp();
		break;
	case 'p':
		if (isPowered()) {
			logger->println("turning motor power off");
			powerRelay.power(false);
		} else {
			logger->println("turning motor power on");
			powerRelay.power(true);
		}
		break;
	case 't':
		kinematics.testKinematics();
		kinematics.testInverseKinematics();
		kinematics.testPerformanceKinematics();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		if (isEnabled())
			logging("enabled.");
		else
			logging("disabled.");
		logging(" speed=(");
		logging(menuSpeedX,2,3);
		logging(",");
		logging(menuSpeedY,2,3);
		logging(") angle=(");
		logging(degrees(menuAngleX),4,0);
		logging(",");
		logging(degrees(menuAngleY),4,0);
		logging(")");

		IMUSample a(IMUSamplePlane(menuAngleX,0), IMUSamplePlane(menuAngleY,0),IMUSamplePlane(0,menuOmega));
		getSpeed(micros(), a, menuMovement);
		menuMovement.print();

		logger->println(" >");
	}
}



