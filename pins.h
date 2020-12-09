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

const struct gpio_pin PIN_LED = {0, 17};	// PA17
const struct gpio_pin PIN_SERIAL_DEBUG_TX = {1, 22};  // PB22
const struct gpio_pin PIN_SERIAL_DEBUG_RX = {1, 23};  // PB23
const uint8_t PAD_SERIAL_DEBUG_TX = 0x1;
const uint8_t PAD_SERIAL_DEBUG_RX = 0x3;
const struct gpio_pin PIN_6 = {0, 20};  // PA20,  TCC0 / WO6 function F
	
const struct gpio_pin PIN_AIN = {0, 2};  // PA02 
