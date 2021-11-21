#pragma once
#include <macros.h>
#include <stdint.h>

#ifndef __PINS_ARDUINO__
#define __PINS_ARDUINO__

#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif

// Pin count
// ----
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (30u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (0u)

extern PinName digitalPinToPinName(pin_size_t P);

// LEDs
// ----
#define PIN_LED     (7u)
#define LED_BUILTIN PIN_LED
#define RGB     (18u)

// SwitchS
// ----
#define PIN_BTN1     (20u)
#define PIN_BTN2     (21u)

// BUZZER
// ----
#define BUZZER     (22u)

// Motor PWM
// ----
#define PWM1A     (8u)
#define PWM1B     (9u)
#define PWM2A     (10u)
#define PWM2B     (11u)

// SERVO
// ----
#define SRV1     (12u)
#define SRV2     (13u)
#define SRV3     (14u)
#define SRV4     (15u)

// Analog pins
// -----------
#define PIN_A0 (26u)
#define PIN_A1 (27u)
#define PIN_A2 (28u)
#define PIN_A3 (29u)

#define ADC3  (29u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;

#define ADC_RESOLUTION 12

// Serial
#define PIN_SERIAL_TX (0ul)
#define PIN_SERIAL_RX (1ul)

// Wire
#define PIN_WIRE_SDA        (2u)
#define PIN_WIRE_SCL        (3u)

#define PIN_WIRE_SDA1       (4u)
#define PIN_WIRE_SCL1       (5u)

#define SERIAL_HOWMANY		1
#define SERIAL1_TX			(digitalPinToPinName(PIN_SERIAL_TX))
#define SERIAL1_RX			(digitalPinToPinName(PIN_SERIAL_RX))

#define SERIAL_CDC			1
#define HAS_UNIQUE_ISERIAL_DESCRIPTOR
#define BOARD_VENDORID		0x2e8a
#define BOARD_PRODUCTID		0x00c0
#define BOARD_NAME			"RaspberryPi Pico"

uint8_t getUniqueSerialNumber(uint8_t* name);
void _ontouch1200bps_();

#define WIRE_HOWMANY	(2)
#define I2C_SDA			(digitalPinToPinName(PIN_WIRE_SDA))
#define I2C_SCL			(digitalPinToPinName(PIN_WIRE_SCL))
#define I2C_SDA1			(digitalPinToPinName(PIN_WIRE_SDA1))
#define I2C_SCL1			(digitalPinToPinName(PIN_WIRE_SCL1))

#define digitalPinToPort(P)		(digitalPinToPinName(P)/32)

#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#define USB_MAX_POWER	(500)

#endif //__PINS_ARDUINO__
