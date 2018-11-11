/*
 * IMUController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <MPU9250/src/MPU9250.h>
#include <TimePassedBy.h>
#include <Filter/KalmanFilter.h>
#include <IMU.h>
#include <setup.h>
#include <Util.h>
#include <types.h>
#include <BotMemory.h>

// flag indicating that IMU has a new measurement, this is set in interrupt
// and evaluated in loop(), so this needs to be declared volatile
volatile bool newDataAvailable = false;

// if the interrupt has been missed, use this emergency timer
// to ask the IMU anyhow.
TimePassedBy updateTimer(2.0*SamplingTime*1000.0 /* [ms] */); // twice the usual sampling frquency


// interrupt that is called whenever MPU9250 has a new value (which is setup'ed to happen every 10ms)
void imuInterrupt() {
	newDataAvailable = true;
}


void IMUConfig::initDefaultValues() {
	nullOffsetX = radians(2.9);
	nullOffsetY = radians(2.20);
	nullOffsetZ = radians(1.18);
	kalmanNoiseVariance = 0.03; // noise variance, default is 0.03, the higher the more noise is filtered
}

void IMUConfig::print() {
	logger->println("imu configuration");
	logger->print("   null=(");
	logger->print(degrees(nullOffsetX));
	logger->print(",");
	logger->print(degrees(nullOffsetY));
	logger->print(",");
	logger->print(degrees(nullOffsetZ));
	logger->println("))");
	logger->print("   kalman noise variance=");
	logger->println(kalmanNoiseVariance,2);

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
	plane[Dimension::X] = x;
	plane[Dimension::Y] = y;
	plane[Dimension::Z] = z;
}

IMUSample::IMUSample(const IMUSample& t) {
	this->plane[0] = t.plane[0];
	this->plane[1] = t.plane[1];
	this->plane[2] = t.plane[2];
}

IMUSample& IMUSample::operator=(const IMUSample& t) {
	this->plane[0] = t.plane[0];
	this->plane[1] = t.plane[1];
	this->plane[2] = t.plane[2];

	return *this;
}


bool IMU::isValid() {
	return ((millis() - updateTimer.mLastCall_ms < 2000/SampleFrequency) &&
			(abs(currentSample.plane[X].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[Y].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[X].angularVelocity) < MaxTiltAngle/SamplingTime) &&
			(abs(currentSample.plane[Y].angularVelocity) < MaxTiltAngle/SamplingTime) &&
			(abs(currentSample.plane[X].angle - lastSample.plane[X].angle) < MaxAngularVelocityAngle) &&
			(abs(currentSample.plane[Y].angle - lastSample.plane[Y].angle) < MaxAngularVelocityAngle));
}

void IMU::setup(MenuController *newMenuCtrl) {
	registerMenuController(newMenuCtrl);

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

	status = init();
}

int IMU::init() {
	// setting the accelerometer full scale range to +/-8G
	mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_2G);

	// setting the gyroscope full scale range to +/-500 deg/s
	int status = mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS);

	// setting low pass bandwith
	status = mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ); // default, kalman filter takes over

	// set update rate of gyro to 200 Hz
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

	kalman[Dimension::X].setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);
	kalman[Dimension::Y].setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);
	kalman[Dimension::Z].setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);

	return status;
}

void IMU::calibrate() {
	logger->println("calibrate imu");
	int status = mpu9250->calibrateAccel();
	if (status != 1) {
		logger->print("accl calibration status error");
		logger->println(status);
		fatalError("fatal error");
	}

	status = mpu9250->calibrateGyro();
	if (status != 1) {
		logger->print("accl calibration status error");
		logger->println(status);
		fatalError("fatal error");
	}

	status = init();

	if (status != 1) {
		logger->print("status error");
		logger->println(status);
		fatalError("fatal error");
	}
	// measure for 1s, run kalman filter and take final orientation as null value
	uint32_t now = millis();
	memory.persistentMem.imuControllerConfig.nullOffsetX = 0;
	memory.persistentMem.imuControllerConfig.nullOffsetY = 0;
	memory.persistentMem.imuControllerConfig.nullOffsetZ = 0;

	while (millis() - now < 1000) {
		loop();
	}

	memory.persistentMem.imuControllerConfig.nullOffsetX = kalman[Dimension::X].getAngle();
	memory.persistentMem.imuControllerConfig.nullOffsetY = kalman[Dimension::Y].getAngle();
	memory.persistentMem.imuControllerConfig.nullOffsetZ = kalman[Dimension::Z].getAngle();

	memory.persistentMem.imuControllerConfig.print();
}

void IMU::loop() {
	if (mpu9250) {
		if (newDataAvailable || updateTimer.isDue()) {
			if (newDataAvailable) {
				updateTimer.dT(); // reset timer of updateTimer
				newDataAvailable = false;
			} else {
				warnMsg("IMU does not send interrupts");
			}

			// read raw values
			int status = mpu9250->readSensor();
			if (status != 1) {
				fatalError("IMU status error");
			}

			// compute dT for kalman filter
			uint32_t now = millis();
			dT = ((float)(now - lastInvocationTime_ms))/1000.0;
			lastInvocationTime_ms = now;

			// turn the coordinate system of the IMU into that one of the bot:
			// front wheel points to the x-axis, y-axis is
			// for use of the kalman filter, we need to break the convention and
			// denote the coordsystem for angualr velocity in the direction of the according axis
			// I.e. the angular velocity in the x-axis denotes the speed of the tilt angle in direction of x
			float tilt[3];
			tilt[Dimension::X] = mpu9250->getAccelX_mss()*(HALF_PI/Gravity);
			tilt[Dimension::Y] = -mpu9250->getAccelY_mss()*(HALF_PI/Gravity);
			tilt[Dimension::Z] =  mpu9250->getAccelZ_mss()*(HALF_PI/Gravity) + HALF_PI;

			float angularVelocity[3];
			angularVelocity[Dimension::X] = mpu9250->getGyroY_rads();
			angularVelocity[Dimension::Y] = mpu9250->getGyroX_rads();
			angularVelocity[Dimension::Z] = mpu9250->getGyroZ_rads();

			// invoke kalman filter separately per plane
			for (int i = 0;i<3;i++)
				kalman[i].update(tilt[i], angularVelocity[i], dT);

			lastSample = currentSample;
			currentSample.plane[Dimension::X].angle = kalman[Dimension::X].getAngle() - memory.persistentMem.imuControllerConfig.nullOffsetX;
			currentSample.plane[Dimension::Y].angle = kalman[Dimension::Y].getAngle() - memory.persistentMem.imuControllerConfig.nullOffsetY;
			currentSample.plane[Dimension::Z].angle = kalman[Dimension::Z].getAngle() - memory.persistentMem.imuControllerConfig.nullOffsetZ;
			currentSample.plane[Dimension::X].angularVelocity = kalman[Dimension::X].getRate();
			currentSample.plane[Dimension::Y].angularVelocity = kalman[Dimension::Y].getRate();
			currentSample.plane[Dimension::Z].angularVelocity = kalman[Dimension::Z].getRate();

			// indicate that new value is available
			valueIsUpdated = true;

			averageTime_ms = (averageTime_ms + (millis() - now))/2;

			if (logIMUValues) {
				command->print("dT=");
				command->print(dT,3);
				command->print("a=(X:");
				command->print(degrees(tilt[Dimension::X]),2);
				command->print("/");
				command->print(degrees(angularVelocity[Dimension::X]),2);
				command->print("Y:");
				command->print(degrees(tilt[Dimension::Y]),2);
				command->print("/");
				command->print(degrees(angularVelocity[Dimension::Y]),2);
				command->print("Z:");
				command->print(degrees(tilt[Dimension::Z]),2);
				command->print("/");
				command->print(degrees(angularVelocity[Dimension::Z]),2);

				command->print(" angle=(");
				command->print(degrees(getAngleRad(Dimension::X)));
				command->print(",");
				command->print(degrees(getAngleRad(Dimension::Y)));
				command->print(")");

				command->print("kalman t=");
				command->print(averageTime_ms);
				command->println("us");
			}
		}
	}
}

float IMU::getAvrLoopTime() {
	return ((float)averageTime_ms)/1000.0;
}


// returns true once when a new value is available.
bool IMU::isNewValueAvailable(float &dT) {
	bool tmp = valueIsUpdated;
	valueIsUpdated = false;
	dT = this->dT;
	return tmp;
}

float IMU::getAngleRad(Dimension dim) {
	return currentSample.plane[dim].angle;
}

float IMU::getAngularVelocity(Dimension dim) {
	return currentSample.plane[dim].angularVelocity;
}

void IMU::setNoiseVariance(float noiseVariance) {
	kalman[0].setNoiseVariance(noiseVariance);
	kalman[1].setNoiseVariance(noiseVariance);
	kalman[2].setNoiseVariance(noiseVariance);
}

void IMU::printHelp() {
	command->println("IMU controller");
	command->println("r    - read values");
	command->println("c    - calibrate ");
	command->println("n/M  - set kalman noise variance");

	command->println("ESC");
}

void IMU::menuLoop(char ch, bool continously) {
	bool cmd = true;
	switch (ch) {
	case 'r':
		logIMUValues = !logIMUValues;
		break;
	case 'h':
		printHelp();
		break;
	case 'c':
		calibrate();
		break;
	case 'N':
		if (memory.persistentMem.imuControllerConfig.kalmanNoiseVariance < 1.0)
			memory.persistentMem.imuControllerConfig.kalmanNoiseVariance += 0.01;
		logger->print("kalman noise variance ");
		logger->println(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance ,2);
		setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);
		break;
	case 'n':
		if (memory.persistentMem.imuControllerConfig.kalmanNoiseVariance > 0.01)
			memory.persistentMem.imuControllerConfig.kalmanNoiseVariance -= 0.01;
		logger->print("kalman noise variance ");
		logger->println(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance ,2);
		setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);
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
