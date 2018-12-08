/*
 * IMUController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <MPU9250/MPU9250.h>
#include <TimePassedBy.h>
#include <Filter/KalmanFilter.h>
#include <IMU.h>
#include <setup.h>
#include <libraries/Util.h>
#include <types.h>
#include <BotMemory.h>
#include <libraries/I2CPortScanner.h>

// instantiated in main.cpp
extern i2c_t3* IMUWire;


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
	// these null values can be calibrated and set in EEPROM
	nullOffsetX = radians(2.9);
	nullOffsetY = radians(2.20);
	nullOffsetZ = radians(1.18);
	kalmanNoiseVariance = 0.03; // noise variance, default is 0.03, the higher the more noise is filtered
}

void IMUConfig::print() {
	loggingln("imu configuration");
	logging("   null=(");
	logging(degrees(nullOffsetX),3,2);
	logging(",");
	logging(degrees(nullOffsetY),3,2);
	logging(",");
	logging(degrees(nullOffsetZ),3,2);
	loggingln("))");
	logging("   kalman noise variance=");
	loggingln(kalmanNoiseVariance,1,3);

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
	if (!(millis() - updateTimer.mLastCall_ms < 2000/SampleFrequency))
		logging("IMU frequency too low");
	if (!(abs(currentSample.plane[X].angle) < MaxTiltAngle))
		logging("X tilt ange too high");
	if (!(abs(currentSample.plane[Y].angle) < MaxTiltAngle))
		logging("Y tilt ange too high");
	if (!(abs(currentSample.plane[X].angularVelocity) < MaxTiltAngle/SamplingTime))
		logging("X angular velocity ange too high");
	if (!(abs(currentSample.plane[Y].angularVelocity) < MaxTiltAngle/SamplingTime))
		logging("Y X angular velocity too high");

	bool result = ((millis() - updateTimer.mLastCall_ms < 2000/SampleFrequency) &&
			(abs(currentSample.plane[X].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[Y].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[X].angularVelocity) < MaxTiltAngle/SamplingTime) &&
			(abs(currentSample.plane[Y].angularVelocity) < MaxTiltAngle/SamplingTime));
	if (!result) {
		logging("t=");
		logging(millis() - updateTimer.mLastCall_ms);
		logging("a=");
		logging(currentSample.plane[X].angle,3,1);
		logging(",");
		logging(currentSample.plane[Y].angle,3,1);
		logging("w=");
		logging(currentSample.plane[X].angularVelocity,3,1);
		logging(",");
		logging(currentSample.plane[Y].angularVelocity,3,1);
		loggingln();
	}
	return result;
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

	// doI2CPortScan(F("I2C"),IMUWire , logger);
	mpu9250 = new MPU9250(IMUWire,IMU_I2C_ADDRESS,I2C_RATE_800);

	int status = mpu9250->begin();
	if (status < 0)
		fatalError("I2C-IMU setup failed ");

	status = init();
	if (status < 0)
		fatalError("I2C-IMU init failed ");

	// clean up if failed
	if (status < 0) {
		if (mpu9250 != NULL)
			delete mpu9250;
		mpu9250 = NULL;
	}
}

int IMU::init() {
	// setting the accelerometer full scale range to +/-2G
	int status =mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_2G);

	// setting the gyroscope full scale range to +/-500 deg/s
	status = mpu9250->setGyroRange(MPU9250::GYRO_RANGE_250DPS);

	// setting low pass bandwith
	// status = mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ); // kalman filter does the rest

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
	loggingln("calibrate imu");
	int status = mpu9250->calibrateAccel();
	if (status != 1) {
		logging("accl calibration status error");
		loggingln(status);
		fatalError("fatal error");
	}

	status = mpu9250->calibrateGyro();
	if (status != 1) {
		logging("accl calibration status error");
		loggingln(status);
		fatalError("fatal error");
	}

	status = init();

	if (status != 1) {
		logging("status error");
		loggingln(status);
		fatalError("fatal error");
	}
	// measure for 1s, run kalman filter and take final orientation as null value
	uint32_t now = millis();
	memory.persistentMem.imuControllerConfig.nullOffsetX = 0;
	memory.persistentMem.imuControllerConfig.nullOffsetY = 0;
	memory.persistentMem.imuControllerConfig.nullOffsetZ = 0;

	// let kalman filter run and calibrate for 1s
	while (millis() - now < 2000) {
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
				sampleRate_ms = (sampleRate_ms + (millis() - updateTimer.mLastCall_ms)) / 2.0;
				updateTimer.dT(); // reset timer of updateTimer
				newDataAvailable = false;
			} else {
				warnMsg("IMU does not send interrupts");
			}

			// read raw values
			int status = mpu9250->readSensor();
			if (status != 1) {
				fatalError("loop IMU status error ");
				loggingln(status);
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
			tilt[Dimension::X] = atan2(mpu9250->getAccelX_mss(), -mpu9250->getAccelZ_mss()) - memory.persistentMem.imuControllerConfig.nullOffsetX;
			tilt[Dimension::Y] = atan2(-mpu9250->getAccelY_mss(), -mpu9250->getAccelZ_mss()) - memory.persistentMem.imuControllerConfig.nullOffsetY;
			tilt[Dimension::Z] =  mpu9250->getAccelZ_mss();

			float angularVelocity[3];
			angularVelocity[Dimension::X] = mpu9250->getGyroY_rads();
			angularVelocity[Dimension::Y] = mpu9250->getGyroX_rads();
			angularVelocity[Dimension::Z] = mpu9250->getGyroZ_rads();

			// invoke kalman filter separately per plane
			kalman[Dimension::X].update(tilt[Dimension::X], angularVelocity[Dimension::X], dT);
			kalman[Dimension::Y].update(tilt[Dimension::Y], angularVelocity[Dimension::Y], dT);
			kalman[Dimension::Z].update(tilt[Dimension::Z], angularVelocity[Dimension::Z], dT);

			lastSample = currentSample;
			currentSample.plane[Dimension::X].angle = kalman[Dimension::X].getAngle() ;
			currentSample.plane[Dimension::Y].angle = kalman[Dimension::Y].getAngle();
			currentSample.plane[Dimension::Z].angle = kalman[Dimension::Z].getAngle();
			currentSample.plane[Dimension::X].angularVelocity = kalman[Dimension::X].getRate();
			currentSample.plane[Dimension::Y].angularVelocity = kalman[Dimension::Y].getRate();
			currentSample.plane[Dimension::Z].angularVelocity = kalman[Dimension::Z].getRate();

			// indicate that new value is available
			valueIsUpdated = true;

			averageTime_ms = (averageTime_ms + (millis() - now))/2;

			if (logIMUValues) {
				if (logTimer.isDue_ms(50,millis())) {
					logging("dT=");
					logging(dT,1,3);
					logging("a=(X:");
					logging(degrees(tilt[Dimension::X]),2,2);
					logging("/");
					logging(degrees(angularVelocity[Dimension::X]),2,2);
					logging("Y:");
					logging(degrees(tilt[Dimension::Y]),2,2);
					logging("/");
					logging(degrees(angularVelocity[Dimension::Y]),2,2);
					logging("Z:");
					logging(degrees(tilt[Dimension::Z]),2,2);
					logging("/");
					logging(degrees(angularVelocity[Dimension::Z]),2,2);

					logging(" angle=(");
					logging(degrees(getAngleRad(Dimension::X)),2,2);
					logging(",");
					logging(degrees(getAngleRad(Dimension::Y)),2,2);
					logging(")");

					logging("kalman t=");
					logging(averageTime_ms);
					logging("us");
					logging("f=");
					logging(1000/sampleRate_ms);
					loggingln("Hz");

				}
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
	loggingln("IMU controller");
	loggingln("r    - read values");
	loggingln("c    - calibrate ");
	loggingln("n/M  - set kalman noise variance");

	loggingln("ESC");
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
		logging("kalman noise variance ");
		loggingln(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance ,1,3);
		setNoiseVariance(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance);
		break;
	case 'n':
		if (memory.persistentMem.imuControllerConfig.kalmanNoiseVariance > 0.01)
			memory.persistentMem.imuControllerConfig.kalmanNoiseVariance -= 0.01;
		logging("kalman noise variance ");
		loggingln(memory.persistentMem.imuControllerConfig.kalmanNoiseVariance ,1,3);
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
		logging("readvalue=");
		logging(logIMUValues);
		loggingln(" >");
	}
}
