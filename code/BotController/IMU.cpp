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
#include <types.h>

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
	// in case of repeated calls of setup, delete old memory
	if (mpu9250 != NULL) {
		delete mpu9250;
	}

	// initialize high speed I2C to IMU
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
	kalman[Dimension::X].setup(0);
	kalman[Dimension::Y].setup(0);
	kalman[Dimension::Z].setup(0);
}


void IMU::calibrate() {
	mpu9250->calibrateAccel();
	mpu9250->calibrateGyro();
	mpu9250->readSensor();

	// reset Kalman filter
	kalman[Dimension::X].setAngle(0);
	kalman[Dimension::Y].setAngle(0);
	kalman[Dimension::Z].setAngle(0);

	// measure for 1s, run kalman filter and take final orientation as null value
	uint32_t now = millis();
	while (millis() - now > 1000) {
		loop();
	}

	// create rotation matrix out of current angles. The inverse is used
	// later on to turn the orientation coming from the IMU
	// such that it is (0,0,0) if in this orientation
	matrix33_t current;
	computeRotationMatrix(kalman[Dimension::X].getAngle(),  kalman[Dimension::Y].getAngle(), kalman[Dimension::Z].getAngle(), current);
	computeInverseMatrix(current, nullMatrix);
}

void IMU::loop() {
	if (mpu9250) {
		if (newDataAvailable || updateTimer.isDue()) {
			if (newDataAvailable) {
				updateTimer.dT(); // reset timer of updateTimer
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

			float tilt[3];
			tilt[Dimension::X] = mpu9250->getAccelX_mss()*(HALF_PI/Gravity);
			tilt[Dimension::Y] = mpu9250->getAccelY_mss()*(HALF_PI/Gravity);
			tilt[Dimension::Z] = mpu9250->getMagZ_uT();

			float angularVelocity[3];
			angularVelocity[Dimension::X] = -mpu9250->getGyroY_rads();
			angularVelocity[Dimension::Y] = mpu9250->getGyroX_rads();
			angularVelocity[Dimension::Z] = -mpu9250->getGyroZ_rads();

			// invoke kalman filter separately per plane
			for (int i = 0;i<3;i++)
				kalman[i].update(tilt[i], angularVelocity[i], dT);

			float filteredTilt[3] = { kalman[Dimension::X].getAngle(), kalman[Dimension::Y].getAngle(),kalman[Dimension::Z].getAngle() };
			float filteredVelocity[3] = { kalman[Dimension::X].getRate(), kalman[Dimension::Y].getRate(),kalman[Dimension::Z].getRate() };

			float filterNulledTilt[3];
			vectorTimesMatrix(filteredTilt, nullMatrix, filterNulledTilt);
			float filteredNulledVelocity[3];
			vectorTimesMatrix(filteredVelocity, nullMatrix, filteredNulledVelocity);


			currentSample.x.angle = filterNulledTilt[Dimension::X];
			currentSample.y.angle = filterNulledTilt[Dimension::Y];
			currentSample.z.angle = filterNulledTilt[Dimension::Z];
			currentSample.x.angularVelocity = filteredNulledVelocity[Dimension::X];
			currentSample.y.angularVelocity = filteredNulledVelocity[Dimension::Y];
			currentSample.z.angularVelocity = filteredNulledVelocity[Dimension::Z];

			// indicate that new value is available
			valueIsUpdated = true;

			uint32_t end = micros();

			uint32_t duration_us = end - now;
			averageTime_us = averageTime_us/2 + duration_us/2;

			if (logIMUValues) {
				command->print("dT=");
				command->print(dT,3);
				command->print("a=(X:");
				command->print(tilt[Dimension::X],6);
				command->print("/");
				command->print(angularVelocity[Dimension::X],6);
				command->print("Y:");
				command->print(tilt[Dimension::Y],6);
				command->print("/");
				command->print(angularVelocity[Dimension::Y],6);
				command->print("Z:");
				command->print(tilt[Dimension::Z],6);
				command->print("/");
				command->print(angularVelocity[Dimension::Z],6);
				command->print(" angle=(");
				command->print(degrees(getAngleRad(Dimension::X)));
				command->print(",");
				command->print(degrees(getAngleRad(Dimension::Y)));
				command->print(")");
				command->print("kalman t=");
				command->print(averageTime_us);
				command->println("us");
			}
		}
	}
}

// returns true once when a new value is available.
bool IMU::isNewValueAvailable(float &dT) {
	bool tmp = valueIsUpdated;
	valueIsUpdated = false;
	return tmp;
}

float IMU::getAngleRad(Dimension dim) {
	return kalman[dim].getAngle();
}

float IMU::getAngularVelocity(Dimension dim) {
	return kalman[dim].getRate();
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
