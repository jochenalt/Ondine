
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

void increaseWithDifferentiableAcceleration(float &speed, float &accel, float newSpeed,float dT) {
	// an acceleration corresponds with the tilt angle of the bot.
	// Since this can not change immediately, the acceleration needs to be ramped up
	float targetSpeed = constrain(newSpeed, speed - MaxBotAccel*dT, speed +  MaxBotAccel*dT);
	targetSpeed = constrain(targetSpeed, -MaxBotSpeed, +MaxBotSpeed);

	// target acceleration required to reach that speed
	float targetAccel = (targetSpeed - speed)/dT;

	// limit the target acceleration to get a linearly growing acceleration
	accel = constrain(targetAccel, accel - MaxBotAccelAccel*dT, accel + MaxBotAccelAccel*dT);

	// compute the speed out of the new acceleration
	speed += accel*dT;
}

void BotMovement::rampUp(const BotMovement& target, float dT) {
	increaseWithDifferentiableAcceleration(speedX, accelX, target.speedX, dT);
	increaseWithDifferentiableAcceleration(speedY, accelY, target.speedY, dT);

	omega = constrain(target.omega, omega - MaxBotOmegaAccel*dT, omega +  MaxBotOmegaAccel*dT);
	omega = constrain(omega, -MaxBotOmega, +MaxBotOmega);
}

