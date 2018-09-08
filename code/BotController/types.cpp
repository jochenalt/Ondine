
#include <Arduino.h>
#include <types.h>
#include <setup.h>

BotMovement::BotMovement() {
		speedX = 0;
		speedY = 0;
		accelX = 0;
		accelY = 0;
		omega = 0;
}

BotMovement::BotMovement(float speedX, float speedY, float omega) {
		this->speedX = speedX;
		this->speedY = speedY;
		this->accelX = accelX;
		this->accelY = accelY;

		this->omega = omega;
}

BotMovement::BotMovement(const BotMovement& t) {
		speedX = t.speedX;
		speedY = t.speedY;
		accelX = t.accelX;
		accelY = t.accelY;
		omega = t.omega;
}

BotMovement& BotMovement::operator=(const BotMovement& t) {
		speedX = t.speedX;
		speedY = t.speedY;
		accelX = t.accelX;
		accelY = t.accelY;
		omega = t.omega;
		return *this;
}

void increaseWithDifferentiableAcceleration(float &currentSpeed, float &currentAccel, float newSpeed,float dT) {
	if (dT > 0) {
		// an acceleration corresponds with the tilt angle of the bot.
		// Since this can not change immediately, the acceleration needs to be ramped up

		// compute max speed we can reach with max acceleration
		float targetSpeed = constrain(newSpeed, currentSpeed - MaxBotAccel*dT, currentSpeed +  MaxBotAccel*dT);
		targetSpeed = constrain(targetSpeed, -MaxBotSpeed, +MaxBotSpeed);

		// what acceleration would be required to get to that speed?
		float targetAccel = (targetSpeed - currentSpeed)/dT;

		// limit changes in that acceleration to make the movement jerkless
		currentAccel = constrain(targetAccel, currentAccel - MaxBotAccelAccel*dT, currentAccel + MaxBotAccelAccel*dT);

		// compute the speed out of the jerkless acceleration
		currentSpeed += currentAccel*dT;
	}
	else {
		currentSpeed = 0;
		currentAccel = 0;
	}
}

void BotMovement::rampUp(const BotMovement& target, float dT) {
	increaseWithDifferentiableAcceleration(speedX, accelX, target.speedX, dT);
	increaseWithDifferentiableAcceleration(speedY, accelY, target.speedY, dT);

	omega = constrain(target.omega, omega - MaxBotOmegaAccel*dT, omega +  MaxBotOmegaAccel*dT);
	omega = constrain(omega, -MaxBotOmega, +MaxBotOmega);
}

