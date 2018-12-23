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
#include <Filter/FIRFilter.h>

#include <IMU.h>
#include <setup.h>
#include <libraries/Util.h>
#include <types.h>
#include <BotMemory.h>
#include <libraries/I2CPortScanner.h>

// instantiated in main.cpp
extern i2c_t3* IMUWire;


IMUConfig& imuConfig = memory.persistentMem.imuControllerConfig;

// interrupt that is called whenever MPU9250 has a new value (which is setup'ed to happen every 10ms)
volatile uint8_t newDataCounter = 0;

inline void imuInterrupt() {
	newDataCounter++;
}


void IMUConfig::initDefaultValues() {
	// these null values can be calibrated and set in EEPROM
	nullOffsetX = radians(-1.0);
	nullOffsetY = radians(3.2);
	kalmanNoiseVariance = 0.03; // noise variance, default is 0.03, the higher the more noise is filtered
}

void IMUConfig::print() {
	loggingln("imu configuration");
	logging("   null=(");
	logging(degrees(nullOffsetX),3,2);
	logging(",");
	logging(degrees(nullOffsetY),3,2);
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
	if (!(abs(currentSample.plane[X].angle) < MaxTiltAngle))
		logging("X tilt ange too high");
	if (!(abs(currentSample.plane[Y].angle) < MaxTiltAngle))
		logging("Y tilt ange too high");
	if (!(abs(currentSample.plane[X].angularVelocity) < MaxTiltAngle/SamplingTime))
		logging("X angular velocity angle too high");
	if (!(abs(currentSample.plane[Y].angularVelocity) < MaxTiltAngle/SamplingTime))
		logging("Y X angular velocity too high");

	bool result = (abs(currentSample.plane[X].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[Y].angle) < MaxTiltAngle) &&
			(abs(currentSample.plane[X].angularVelocity) < MaxTiltAngle/SamplingTime) &&
			(abs(currentSample.plane[Y].angularVelocity) < MaxTiltAngle/SamplingTime);
	if (!result) {
		logging("a=");
		logging(degrees(currentSample.plane[X].angle),3,1);
		logging(",");
		logging(degrees(currentSample.plane[Y].angle),3,1);
		logging("w=");
		logging(degrees(currentSample.plane[X].angularVelocity),3,1);
		logging(",");
		logging(degrees(currentSample.plane[Y].angularVelocity),3,1);
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
	IMUWire->begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_1200);
	IMUWire->setDefaultTimeout(4000); // 4ms default timeout

	// doI2CPortScan(F("I2C"),IMUWire , logger);
#ifdef FIFO
	mpu9250 = new MPU9250FIFO(IMUWire,IMU_I2C_ADDRESS);
#else
	mpu9250 = new MPU9250(IMUWire,IMU_I2C_ADDRESS);

#endif
	int status = mpu9250->begin();
	if (status < 0) {
		fatalError("I2C-IMU setup failed ");
	}
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
	// enable FIFO mode
#ifdef FIFO
	int status = mpu9250->enableFifo(true,true,false,false);
#else
	int status = 0;
#endif
	enabled = true;

	// setting the accelerometer full scale range to +/-2G
	status = status | mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_2G);

	// setting the gyroscope full scale range to +/-500 deg/s
	status = status | mpu9250->setGyroRange(MPU9250::GYRO_RANGE_250DPS);

	// setting low pass bandwith
	status = mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ); // kalman filter does the rest

	// set update rate of IMU to 200 Hz
	// the interrupt indicating new data will fire with that frequency
#ifdef FIFO
	status = status | mpu9250->setSrd(1000/IMUSamplingFrequency-1); // datasheet: Data Output Rate = 1000 / (1 + SRD)*
#else
	status = status | mpu9250->setSrd(1000/SampleFrequency-1); // datasheet: Data Output Rate = 1000 / (1 + SRD)*

#endif
	mpu9250->setGyroBiasX_rads(0);
	mpu9250->setGyroBiasY_rads(0);
	mpu9250->setGyroBiasZ_rads(0);

	mpu9250->setAccelCalX(0,1.0);
	mpu9250->setAccelCalY(0,1.0);
	mpu9250->setAccelCalZ(0,1.0);

	mpu9250->setMagCalX(0.0, 1.0);
	mpu9250->setMagCalY(0.0, 1.0);
	mpu9250->setMagCalZ(0.0, 1.0);

	// enable aforementioned interrupt
	attachInterrupt(IMU_INTERRUPT_PIN, imuInterrupt, RISING);
	mpu9250->enableDataReadyInterrupt();

	// initialize Kalman filter
	kalman[Dimension::X].setup(0);
	kalman[Dimension::Y].setup(0);
	kalman[Dimension::Z].setup(0);

	kalman[Dimension::X].setNoiseVariance(imuConfig.kalmanNoiseVariance);
	kalman[Dimension::Y].setNoiseVariance(imuConfig.kalmanNoiseVariance);
	kalman[Dimension::Z].setNoiseVariance(imuConfig.kalmanNoiseVariance);

#ifdef FIFO
	const int taps = IMUSamplesPerLoop;
	const float CutOffFrequency = SampleFrequency;
	accelFilter[Dimension::X].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency);
	accelFilter[Dimension::Y].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency );
	accelFilter[Dimension::Z].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency);
	// gyroFilter[Dimension::X].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency);
	// gyroFilter[Dimension::Y].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency);
	// gyroFilter[Dimension::Z].init(FIR::LOWPASS,taps, IMUSamplingFrequency, CutOffFrequency);
	gyroFilter[Dimension::X].init(IMUSamplesPerLoop);
	gyroFilter[Dimension::Y].init(IMUSamplesPerLoop);
	gyroFilter[Dimension::Z].init(IMUSamplesPerLoop);
#endif

	timeLoop.init();

	return status;
}

void IMU::calibrate() {
	/*
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
	// init();
	*/
	// measure for 1s, run kalman filter and take final orientation as null value
	uint32_t now = millis();
	imuConfig.nullOffsetX = 0;
	imuConfig.nullOffsetY = 0;


	// let kalman filter run and calibrate for 1s
	while (millis() - now < 2000) {
		loop(micros());
	}

	imuConfig.nullOffsetX = kalman[Dimension::X].getAngle();
	imuConfig.nullOffsetY = kalman[Dimension::Y].getAngle();

	imuConfig.print();
}

void IMU::loop(uint32_t now_us) {
	if (mpu9250 && enabled &&
#ifdef FIFO
			(newDataCounter >= IMUSamplesPerLoop) &&
#endif
			(newDataCounter > 0) && ((now_us - timeLoop.lastCall_us) >= SampleTime_us)) {

			newDataCounter = 0;
			dT = timeLoop.dT(now_us);

#ifdef FIFO
			// read raw values
			int status = mpu9250->readFifo();
			if (status != 1) {
				fatalError("loop IMU status error ");
				loggingln(status);
			}

			// fetch all IMU samples since the last loop and filter them by average low pass
			int noOfSamples = mpu9250->getFifoSize();

			// check if FIFO buffer did not overflow.
			// FIFO needs to be reinitialized then, other wise there's rubbish data
			if (mpu9250->fifoOverflow()) {

				// reset and re-initialize FIFO
				mpu9250->resetFifo();
				status = mpu9250->enableFifo(true, true, false, false);

				// try again, maybe we get another value
				// (if not, this loop is lost)
				status = mpu9250->readFifo();
				noOfSamples = mpu9250->getFifoSize();
				if (noOfSamples == 0)
					fatalError("IMU lost loop");
			}

			if (noOfSamples > 0) {
				float accelSamples[3][noOfSamples]; // dont be scared, this has typically a size of 3*4=12
				float gyroSamples[3][noOfSamples];

				size_t noOfSamplesDummy;
				// fetch all samples from FIFO queue read already

				for (int dim = 0;dim<3;dim++) {
					mpu9250->getFifoAccel_mss(dim, &noOfSamplesDummy,&accelSamples[dim][0]);
					mpu9250->getFifoGyro_rads(dim, &noOfSamplesDummy,&gyroSamples[dim][0]);
					for (int i = 0;i<noOfSamples;i++) {
						accelFilter[dim].update(accelSamples[dim][i]);
						gyroFilter[dim].update(gyroSamples[dim][i]);
					}
				}
			}
#else
		// read raw values
		delayMicroseconds(100);
		int status = mpu9250->readSensor();
		if (status != 1) {
			fatalError("loop IMU status error ");
			loggingln(status);
		}

		// turn the coordinate system of the IMU into the one of the bot:
		// front wheel points to the x-axis, y-axis is
		// for use of the kalman filter, we need to break the convention and
		// denote the coordsystem for angular velocity in the direction of the according axis
		// I.e. the angular velocity in the x-axis denotes the speed of the tilt angle in direction of x
		float accelX = mpu9250->getAccelX_mss();
		float accelY = mpu9250->getAccelY_mss();
		float accelZ = mpu9250->getAccelZ_mss();

		float angularVelocity[3] = {
					mpu9250->getGyroY_rads(),
					-mpu9250->getGyroX_rads(),
					mpu9250->getGyroZ_rads() };
#endif

		float tilt[3] = {
					atan2f(-accelX, sqrt(accelZ*accelZ + accelY*accelY)) - imuConfig.nullOffsetX,
					atan2f(-accelY, sqrt(accelZ*accelZ + accelX*accelX)) - imuConfig.nullOffsetY,
					accelZ };
		for (int i = 0;i<3;i++) {
			// invoke kalman filter per plane
			kalman[i].update(tilt[i], angularVelocity[i], dT);
			currentSample.plane[i].angle = kalman[i].getAngle();
			currentSample.plane[i].angularVelocity = kalman[i].getRate();
		}

		// indicate that new value is available
		// next call of isNewValueAvailable will return true
		// (but only once)
		valueIsUpdated = true;

		if (logIMUValues) {
			if (logTimer.isDue_ms(50,millis())) {
				logging("dT=");
				logging(((float)(micros()-now_us))/1000.0,2);
				logging("a=(X:");
				logging(degrees(tilt[Dimension::X]),2,2);
				logging("/");
				logging(degrees(angularVelocity[Dimension::X]),2,2);
				logging(" Y:");
				logging(degrees(tilt[Dimension::Y]),2,2);
				logging("/");
				logging(degrees(angularVelocity[Dimension::Y]),2,2);
				logging(" Z:");
				logging(degrees(tilt[Dimension::Z]),2,2);
				logging("/");
				logging(degrees(angularVelocity[Dimension::Z]),2,2);
				logging(" angle=(");
				logging(degrees(getAngleRad(Dimension::X)),2,2);
				logging(",");
				logging(degrees(getAngleRad(Dimension::Y)),2,2);
				logging(")");
#ifdef FIFO
				logging(" #=");
				logging(noOfSamples);
#endif
				logging(" f=");
				logging(timeLoop.getAverageFrequency());
				loggingln("Hz");
			}
		}
	}
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
		if (imuConfig.kalmanNoiseVariance < 1.0)
			imuConfig.kalmanNoiseVariance += 0.01;
		logging("kalman noise variance ");
		loggingln(imuConfig.kalmanNoiseVariance ,1,3);
		setNoiseVariance(imuConfig.kalmanNoiseVariance);
		break;
	case 'n':
		if (imuConfig.kalmanNoiseVariance > 0.01)
			imuConfig.kalmanNoiseVariance -= 0.01;
		logging("kalman noise variance ");
		loggingln(imuConfig.kalmanNoiseVariance ,1,3);
		setNoiseVariance(imuConfig.kalmanNoiseVariance);
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
