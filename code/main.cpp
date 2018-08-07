#include "Arduino.h"
#include "BLDCController.h"

#include <utilities/PatternBlinker.h>

#define LED_PIN 13			// blinking LED on Teensy
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

#define ENCODERA_PIN 10
#define ENCODERB_PIN 11

#define L6234_ENABLE_PIN 2
#define L6234_PWM1 3
#define L6234_PWM2 4
#define L6234_PWM3 5


PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

BLDCController ctrl;


void setup()
{
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	Serial1.begin(115200);
	Serial1.println("startup");

	ctrl.setupMotor(L6234_ENABLE_PIN, L6234_PWM1, L6234_PWM2, L6234_PWM3);
	// ctrl.setupEncoder(ENCODERA_PIN, ENCODERB_PIN, 1024);

	Serial1.println("setup done");

	ctrl.runMenu();
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch

	ctrl.loop();


}
