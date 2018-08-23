/*
 * IMUController.h
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#ifndef IMU_IMUCONTROLLER_H_
#define IMU_IMUCONTROLLER_H_

#include <MenuController.h>
#include <MPU9250/src/MPU9250.h>
#include <Kalman/Kalman.h>

class IMUController : public Menuable {
public:
	virtual ~IMUController() {};
	IMUController() {};

	IMUController& getInstance() {
		static IMUController instance;
		return instance;
	}

	void setup(MenuController* menuCtrl);
	void loop();

	// stateful method to indicate that a new IMU is available. Internal flag
	// is reset such that second call returns false until the next value is available
	bool newValueAvailable();

	float getAngleXRad();
	float getAngleYRad();
	float getAngleZRad();

	// call when stable and upright before starting up
	void calibrate();
	virtual void printHelp();
	virtual void menuLoop(char ch);

private:
	void updateFilter();
	MPU9250* mpu9250 = 0;
	Kalman filterX;
	Kalman filterY;
	Kalman filterZ;

	bool valueIsUpdated = false;
	bool menuPrintValues = false;
	uint32_t lastInvocationTime_ms = 0;
	uint32_t averageTime_ms = 0;
};

#endif /* IMU_IMUCONTROLLER_H_ */
