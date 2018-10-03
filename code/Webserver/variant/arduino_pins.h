#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#define LED_BUILTIN 16

static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

static const uint8_t GPIO16 = D0
static const uint8_t GPIO05 = D1
static const uint8_t GPIO04 = D2
static const uint8_t GPIO00 = D3
static const uint8_t GPIO02 = D4
static const uint8_t GPIO14 = D5
static const uint8_t GPIO12 = D6
static const uint8_t GPIO13 = D7
static const uint8_t GPIO15 = D8
static const uint8_t GPIO03 = D9
static const uint8_t GPIO01 = D10
static const uint8_t GPIO10 = D12
static const uint8_t GPIO09 = D11
static const uint8_t GPIO09 = D11


static const uint8_t TXD1 = GPIO02
static const uint8_t RXD2 = GPIO13
static const uint8_t RXD0 = GPIO15
static const uint8_t TXD0 = GPIO01

#define PIN_WIRE_SDA (D2)
#define PIN_WIRE_SCL (D1)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


#include "../generic/common.h"

#endif /* Pins_Arduino_h */
