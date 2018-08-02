/*
 * EncoderController.cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <EncoderController.h>

EncoderController::EncoderController() {

}

EncoderController::~EncoderController() {
}


#define ENCODERA_PIN 5
#define ENCODERB_PIN 6



void EncoderController::setup()
{
	analogWriteResolution(8);
	analogWriteFrequency(3, 300000);
}
