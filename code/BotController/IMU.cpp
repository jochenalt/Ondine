/*
 * IMUController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <MPU9250/src/MPU9250.h>
#include <utilities/TimePassedBy.h>
#include <Filter/KalmanFilter.h>
#include <IMU.h>
#include <setup.h>
#include <Util.h>
#include <I2CPortScanner.h>


volatile bool newDataAvailable = false;
TimePassedBy updateTimer(SamplingTime*2000.0 /* [ms] */); // emergency timer, in case interrupt did not work



// interrupt that is called whenever MPU9250 has a new value (which is set to happens every 10ms)
void imuInterrupt() {
	if (newDataAvailable) {
	} else {
		newDataAvailable = true;
	}
}

IMUSamplePlane::IMUSamplePlane() {
	angle = 0;
	angularVelocity = 0;
}

IMUSamplePlane::IMUSamplePlane(float angle, float angularVelocity) {
	this->angle = angle;
	this->angularVelocity = angularVelocity;
}

IMUSamplePlane::IMUSamplePlane(const IMUSamplePlane& t) {
	this->angle = t.angle;
	this->angularVelocity = t.angularVelocity;
}

IMUSamplePlane& IMUSamplePlane::operator=(const IMUSamplePlane& t) {
	this->angle = t.angle;
	this->angularVelocity = t.angularVelocity;
	return * this;
}

IMUSample::IMUSample() {};

IMUSample::IMUSample(const IMUSamplePlane& x, const IMUSamplePlane& y, const IMUSamplePlane& z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

IMUSample::IMUSample(const IMUSample& t) {
	this->x = t.x;
	this->y = t.y;
	this->z = t.z;

}

IMUSample& IMUSample::operator=(const IMUSample& t) {
	this->x = t.x;
	this->y = t.y;
	this->z = t.z;
	return *this;
}


void IMU::setup(MenuController *newMenuCtrl) {
	registerMenuController(newMenuCtrl);
}

void IMU::setup() {
	if (mpu9250 != NULL) {
		delete mpu9250;
	}
	IMUWire = &Wire;
	IMUWire->begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_800);
	IMUWire->setDefaultTimeout(4000); // 4ms default timeout

	mpu9250 = new MPU9250(IMUWire,IMU_I2C_ADDRESS,I2C_RATE_400);
	int status = mpu9250->begin();
	if (status < 0) {
		fatalError("I2C-IMU setup failed ");
	}

	// setting the accelerometer full scale range to +/-8G
	mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_8G);

	// setting the gyroscope full scale range to +/-500 deg/s
	status = mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS);

	// setting low pass bandwith
	status = mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);

	// set update rate of gyro to 100 Hz
	status = mpu9250->setSrd(1000/SampleFrequency-1); // datasheet: Data Output Rate = 1000 / (1 + SRD)*

	mpu9250->setGyroBiasX_rads(0);
	mpu9250->setGyroBiasY_rads(0);
	mpu9250->setGyroBiasZ_rads(0);

	mpu9250->setAccelCalX(0,1.0);
	mpu9250->setAccelCalY(0,1.0);
	mpu9250->setAccelCalZ(0,1.0);

	mpu9250->setMagCalX(0.0, 1.0);
	mpu9250->setMagCalY(0.0, 1.0);
	mpu9250->setMagCalZ(0.0, 1.0);

	// enable interrupt
	attachInterrupt(IMU_INTERRUPT_PIN, imuInterrupt, RISING);
	mpu9250->enableDataReadyInterrupt();

	// initialize Kalman filter
	filterX.setup(0);
	filterY.setup(0);
	filterZ.setup(0);
}


void IMU::calibrate() {
	mpu9250->calibrateAccel();
	mpu9250->calibrateGyro();
	mpu9250->readSensor();

	// reset Kalman filter
	filterX.setAngle(0);
	filterY.setAngle(0);
	filterZ.setAngle(0);
}

void IMU::loop() {
	if (newDataAvailable || updateTimer.isDue()) {
		if (newDataAvailable) {
			updateTimer.dT(); // reset timer
			newDataAvailable = false;
		} else {
			warnMsg("interrupt of IMU not working");
		}

		// read raw values
		int status = mpu9250->readSensor();
		if (status != 1) {
			fatalError("IMU status error");
		}

		// compute dT for kalman filter
		uint32_t now = micros();
		dT = ((float)(now - lastInvocationTime_ms))/1000000.0;
		lastInvocationTime_ms = now;

		float tiltX = mpu9250->getAccelX_mss()*(HALF_PI/Gravity);
		float tiltY = mpu9250->getAccelY_mss()*(HALF_PI/Gravity);
		float tiltZ = mpu9250->getMagZ_uT();

		float angularVelocityX = -mpu9250->getGyroY_rads();
		float angularVelocityY = mpu9250->getGyroX_rads();
		float angularVelocityZ = -mpu9250->getGyroZ_rads();

		// invoke kalman filter separately per plane
		filterX.update(tiltX, angularVelocityX, dT);
		filterY.update(tiltY, angularVelocityY, dT);
		filterZ.update(tiltZ, angularVelocityZ, dT);

		// indicate that new value is available
		valueIsUpdated = true;

		uint32_t end = micros();

		uint32_t duration_us = end - now;
		averageTime_us = averageTime_us/2 + duration_us/2;

		if (logIMUValues) {
			command->print("dT=");
			command->print(dT,3);
			command->print("a=(X:");
			command->print(tiltX,6);
			command->print("/");
			command->print(angularVelocityX,6);
			command->print("Y:");
			command->print(tiltY,6);
			command->print("/");
			command->print(angularVelocityY,6);
			command->print("Z:");
			command->print(tiltZ,6);
			command->print("/");
			command->print(angularVelocityZ,6);
			command->print(" angle=(");
			command->print(degrees(getAngleXRad()));
			command->print(",");
			command->print(degrees(getAngleYRad()));
			command->print(")");
			command->print("kalman t=");
			command->print(averageTime_us);
			command->println("us");
		}
	}
}

// returns true once when a new value is available.
bool IMU::isNewValueAvailable(float &dT) {
	bool tmp = valueIsUpdated;
	valueIsUpdated = false;
	return tmp;
}

float IMU::getAngleXRad() {
	return filterX.getAngle();
}

float IMU::getAngleYRad() {
	return filterY.getAngle();
}

float IMU::getAngleZRad() {
	return filterZ.getAngle();
}

float IMU::getAngularVelocityX() {
	return filterX.getRate();
}

float IMU::getAngularVelocityY() {
	return filterY.getRate();
}

float IMU::getAnglularVelocityZ() {
	return filterZ.getRate();
}

void IMU::printHelp() {
	command->println("IMU controller");
	command->println("s - setup");
	command->println("r - read values");
	command->println("c - calibrate ");

	command->println("ESC");
}

void IMU::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 'r':
		logIMUValues = !logIMUValues;
		break;
	case 's':
		setup();
		break;
	case 'h':
		printHelp();
		break;
	case 'c':
		calibrate();
		break;
	case 27:
		popMenu();
		return;
		break;
	default:
		cmd = false;
		break;
	}


	if (cmd) {
		command->print("readvalue=");
		command->print(logIMUValues);
		command->println(" >");

	}
}
