#include "Arduino.h"
#include "BLDCController.h"
#include "MenuController.h"
#include "Engine.h"

#include <utilities/PatternBlinker.h>

#define LED_PIN 13			// blinking LED on Teensy
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

#define ENCODERA_PIN 6
#define ENCODERB_PIN 7

#define L6234_ENABLE_PIN 2
#define L6234_PWM1 3
#define L6234_PWM2 4
#define L6234_PWM3 5


PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

Engine engine;
MenuController menuCtrl;

void setup()
{
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	Serial1.begin(115200);
	Serial1.println("startup");

	engine.setup(menuCtrl);

	Serial1.println("setup done");
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch

	TimePassedBy timer(1);
	if (timer.isDue())
		engine.loop();

	menuCtrl.loop();

}
