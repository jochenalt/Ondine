#include "Arduino.h"

#define ENCODER_USE_INTERRUPTS
#include <Encoder/Encoder.h>
#include <utilities/PatternBlinker.h>

#define LED_PIN 13			// blinking LED on Teensy
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

#define ENCODERA_PIN 5
#define ENCODERB_PIN 6

PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(ENCODERA_PIN, ENCODERB_PIN);
long oldPosition  = -999;


void setup()
{
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	Serial1.begin(115200);
	Serial1.println("startup1");

	analogWriteResolution(8);
	analogWriteFrequency(3, 300000);
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch
	/*

	  long newPosition = myEnc.read();
	  if (newPosition != oldPosition) {
	    oldPosition = newPosition;
	    Serial1.print("new position");
	    Serial1.println(newPosition);
	  }
	 */
	static int c = 0;
	static uint32_t lastTime  = 0;
	if (now - lastTime > 2000) {
		Serial1.println("now");
		Serial1.print("0");
		analogWrite(3, 0);
		delay(2000);
		analogWrite(3, 255);
		Serial1.print("255");
		delay(2000);
		analogWrite(3, 64);
		Serial1.print("64");
		delay(2000);

		Serial1.print("now ");
		Serial1.print(c % 256);
		Serial1.println();

		lastTime = now;
	}
}
