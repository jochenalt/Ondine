
#ifndef TYPES_H_
#define TYPES_H_


class BotMovement {
public:
	BotMovement();
	BotMovement(float speedX, float speedY, float omega);
	BotMovement(const BotMovement& t);
	BotMovement& operator=(const BotMovement& t);

	// increase movement towards target in a jerkless manner,
	// i.e. the acceleration used increases with a constant rate
	void rampUp(const BotMovement& target, float dT);

	float speedX = 0;
	float speedY = 0;
	float accelX = 0;
	float accelY = 0;
	float omega = 0;
};

#endif
