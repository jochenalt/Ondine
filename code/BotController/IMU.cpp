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


volatile bool newDataAvailable = false;
TimePassedBy updateTimer(20);



// interrupt that is called whenever MPU9250 has a new value (which is set to happens every 10ms)
void imuInterrupt() {
	newDataAvailable = true;
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

	// i2c frequency
	Wire.setClock(400000);

	mpu9250 = new MPU9250(Wire,0x68 /* I2C address */);
	int status = mpu9250->begin();
	if (status < 0) {
	    Serial.print("IMU setup failed ");
	    Serial.println(status);
	}

	// setting the accelerometer full scale range to +/-8G
	mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_8G);

	// setting the gyroscope full scale range to +/-500 deg/s
	status = mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS);

	// setting low pass bandwith
	status = mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);

	// set update rate of gyro to 100 Hz
	status = mpu9250->setSrd(1000/SampleFrequency-1); // datasheet: Data Output Rate = 1000 / (1 + SRD)*

	// enable interrupt
	mpu9250->enableDataReadyInterrupt();

	attachInterrupt(IMU_INTERRUPT_PIN, imuInterrupt, RISING);

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
		uint32_t start = millis();

		// read raw values
		mpu9250->readSensor();

		// compute dT for kalman filter
		uint32_t now = millis();
		dT = ((float)(now - lastInvocationTime_ms))/1000.0;
		lastInvocationTime_ms = now;

		// invoke kalman filter separately per plane
		filterX.update(mpu9250->getAccelX_mss(), mpu9250->getGyroX_rads(), dT);
		filterY.update(mpu9250->getAccelY_mss(), mpu9250->getGyroY_rads(), dT);
		filterZ.update(mpu9250->getMagZ_uT(), mpu9250->getGyroZ_rads(), dT);

		// indicate that new value is available
		valueIsUpdated = true;

		uint32_t end = millis();

		uint32_t duration_ms = end - start;
		averageTime_ms += duration_ms;
		averageTime_ms /= 2;
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
	command->println("r - read values");
	command->println("c - calibrate ");

	command->println("ESC");
}

void IMU::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 'r':
		menuPrintValues = !menuPrintValues;
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

	if (menuPrintValues) {
		command->print("angle=(");
		command->print(degrees(getAngleXRad()));
		command->print(",");
		command->print(degrees(getAngleYRad()));
		command->println(")");
	}
	if (cmd) {
		command->print("read IMU values");
		command->println(" >");
	}
}
