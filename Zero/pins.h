/*
 * Pins.h
 *
 * Created: 28.11.2020 08:00:00
 * Author : kozak
 */ 

#include "sam.h"

const uint32_t XOSC32K_freq = 32768;

struct gpio_pin {
	uint8_t	group;
	uint8_t pin;
};

struct serial_pins {
	struct gpio_pin tx;
	struct gpio_pin rx;
};

const struct gpio_pin PIN_LED = {0, 17};	// PA17
const struct gpio_pin PIN_SERIAL_DEBUG_TX = {1, 22};  // PB22
const struct gpio_pin PIN_SERIAL_DEBUG_RX = {1, 23};  // PB23
	

Sercom* SERCOM_DEBUG = SERCOM5;
