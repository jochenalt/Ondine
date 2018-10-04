#ifndef _PINS_H
#define _PINS_H

#include <pins_arduino.h>
#define LED_BUILTIN 16

static const uint8_t GPIO16 = D0;
static const uint8_t GPIO05 = D1;
static const uint8_t GPIO04 = D2;
static const uint8_t GPIO00 = D3;
static const uint8_t GPIO02 = D4;
static const uint8_t GPIO14 = D5;
static const uint8_t GPIO12 = D6;
static const uint8_t GPIO13 = D7;
static const uint8_t GPIO15 = D8;
static const uint8_t GPIO03 = D9;
static const uint8_t GPIO01 = D10;

// Uart used for receiving log entries of bot controller
static const uint8_t RXD0 = GPIO03;
static const uint8_t TXD0 = GPIO01;

// currently not used
static const uint8_t RXD2 = GPIO13;
static const uint8_t TXD2 = GPIO15;


#endif /* Pins_Arduino_h */
