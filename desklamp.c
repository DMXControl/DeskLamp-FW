/**
 * @file 	desklamp.c
 * @author  J. Mintenbeck
 * @date    2013/11/09
 * @version 1.0
 * @brief	Desklamp API
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "desklamp.h"
#include "entropy.h"

#define SERIAL_EEPROM_STORE		0
#define COLORMODE_EEPROM_STORE	4
#define ADAPTER_EEPROM_STORE	5

static desklamp_t desklamp;


/** brightness table */
const PROGMEM uint8_t pwmtable[128] = {
		0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
		1, 1, 1, 2, 2, 2, 3, 3, 3, 4,
		4, 5, 5, 6, 7, 7, 8, 8, 9, 10,
		11, 11, 12, 13, 14, 15, 16, 17,
		18, 19, 20, 21, 22, 24, 25, 26,
		27, 29, 30, 31, 33, 34, 36, 37,
		39, 40, 42, 44, 45, 47, 49, 51,
		53, 55, 56, 58, 60, 62, 65, 67,
		69, 71, 73, 75, 78, 80, 82, 85,
		87, 90, 92, 95, 97, 100, 103,
		105, 108, 111, 114, 117, 120,
		122, 125, 128, 132, 135, 138,
		141, 144, 147, 151, 154, 157,
		161, 164, 168, 171, 175, 179,
		182, 186, 190, 193, 197, 201,
		205, 209, 213, 217, 221, 225,
		229, 233, 238, 242, 246, 251,
		255
};

/**
 *  @brief      Init Pins of Desklamp
*/
void desklamp_init(void) {
	// setup desklamp
	desklamp.state = DESKLAMP_STATE_IDLE;
	desklamp.strobe = 0; // Strobe = 0 Hz
	desklamp.blackout = 0;
	desklamp.usb_ext = 1;

	desklamp.r = 255;
	desklamp.g = 255;
	desklamp.b = 255;
	desklamp.dimmer = 255;

	desklamp.colormode = eeprom_read_byte((unsigned char *)COLORMODE_EEPROM_STORE);
	if (desklamp.colormode == 0xFF) {
		desklamp_set_colormode(COLORMODE);
	}

	desklamp.isAdapter = eeprom_read_byte((unsigned char *)ADAPTER_EEPROM_STORE);
	if (desklamp.isAdapter == 0xFF) {
		desklamp_set_adapter(USBADAPTER);
	}

	eeprom_read_block(&desklamp.serial, (unsigned char *)SERIAL_EEPROM_STORE, 4);
	if (desklamp.serial == 0xFFFFFFFF) {
		entropy_init();
		desklamp_set_serial(entropy_random());
	}

	// set Ext USB PIN as Input
	DESKLAMP_LED_DDR &= ~(1 << DESKLAMP_PIN_DP_EXT);
	DESKLAMP_LED_DDR &= ~(1 << DESKLAMP_PIN_DM_EXT);

	// set LED-ports to output
	DESKLAMP_LED_DDR |= (1 << DESKLAMP_PIN_LED1);
	DESKLAMP_LED_DDR |= (1 << DESKLAMP_PIN_LED2);
	DESKLAMP_LED_DDR |= (1 << DESKLAMP_PIN_LED3);

	// switch leds on
	desklamp_set_led(1, ON);
	if (desklamp.colormode == DESKLAMP_COLORMODE_RGB) {
		desklamp_set_led(2, ON);
		desklamp_set_led(3, ON);
	}
}

/**
 *  @brief      Config PWM Channels
 *  @param    	channel Channel number (1..3)
 *  @param    	state 	ENABLE, DISABLE
*/
void desklamp_config_channel(uint8_t channel, uint8_t state){
	switch (channel) {
		case 1:												// Channel 1
			if (state == ENABLE)
				TCCR0A |= (1 << COM0B1);					// Non-Inverting PWM
			else {
				TCCR0A &= ~(1 << COM0B1);					// Normal Pin operation
			}
			break;
		case 2:												// Channel 2
			if (state == ENABLE)
				TCCR1A |= (1 << COM1A1);					// Non-Inverting PWM
			else {
				TCCR1A &= ~(1 << COM1A1);					// Normal Pin operation
			}
			break;

		case 3:												// Channel 3
			if (state == ENABLE)
				TCCR1A |= (1 << COM1B1);					// Non-Inverting PWM
			else {
				TCCR1A &= ~(1 << COM1B1);					// Normal Pin operation
			}
			break;
	}
}

/**
 *  @brief      Init PWM-Hardware
*/
void desklamp_init_pwm(void) {
	// init timers as fast PWM
	TCCR0A |= (1 << WGM00) | (1 << WGM01);		// Fast PWM Mode 3
	TCCR1A |= (1 << WGM10) | (1 << WGM12);		// Fast PWM Mode 5 (8bit)
	ICR1 = 0xff;

	// set prescaler 256
	TCCR0B |= (1 << CS02);						// prescaler 256 	-> 183,10546875 Hz
	if (desklamp.colormode == DESKLAMP_COLORMODE_RGB) {
		TCCR1B |= (1 << CS12);					// prescaler 256	->  90Hz
	}

	// set outputs to PWM
	desklamp_config_channel(1, ENABLE);			// Non-Inverting PWM 1
	if (desklamp.colormode == DESKLAMP_COLORMODE_RGB) {
		desklamp_config_channel(2, ENABLE);		// Non-Inverting PWM 2
		desklamp_config_channel(3, ENABLE);		// Non-Inverting PWM 3
	}

	desklamp_update_pwm();
}



/** SET Functions */

/**
 *  @brief      Set LED on/off
 *  @param    	led (1..3)
 *  @param    	onoff (ON, OFF)
*/
void desklamp_set_led(uint8_t led, uint8_t onoff) {
	uint8_t ledid = 1;

	switch (led) {
		case 1:
			ledid = DESKLAMP_PIN_LED1;
			break;

		case 2:
			ledid = DESKLAMP_PIN_LED2;
			break;

		case 3:
			ledid = DESKLAMP_PIN_LED3;
			break;
	}

	switch (onoff) {
		case ON:
			DESKLAMP_LED_PORT |= (1<< ledid);
			break;

		case OFF:
			DESKLAMP_LED_PORT &= ~(1<< ledid);
			break;
	}

}

/**
 *  @brief      Set LED intensity
 *  @param    	led (1..3)
 *  @param    	intensity (0..255)
*/
void desklamp_set_led_intensity(uint8_t led, uint8_t intensity){
	switch (led) {
		case 1:		// led red or single
			desklamp.r = intensity;
			break;

		case 2:		// led green
			desklamp.g = intensity;
			break;

		case 3:		// led blue
			desklamp.b = intensity;
			break;
	}
	desklamp_update_pwm();
}

/**
 *  @brief      Set RGB Value
 *  @param    	r red (0..255)
 *  @param    	g green (0..255)
 *  @param    	b blue (0..255)
*/
void desklamp_set_rgb(uint8_t r, uint8_t g, uint8_t b){
	desklamp.r = r;
	desklamp.g = g;
	desklamp.b = b;

	desklamp_update_pwm();
}

/**
 *  @brief      Set desklamp dimmer
 *  @param    	dimmer (0..255)
*/
void desklamp_set_dimmer(uint8_t dimmer){
	desklamp.dimmer = dimmer;

	desklamp_update_pwm();
}

/**
 *  @brief      Set Colormode
 *  @param    	colormode (RGB, MONO)
*/
void desklamp_set_colormode(uint8_t colormode){
	switch (colormode) {
		case DESKLAMP_COLORMODE_RGB:
			desklamp.colormode = DESKLAMP_COLORMODE_RGB;
			break;
		case DESKLAMP_COLORMODE_MONO:
			desklamp.colormode = DESKLAMP_COLORMODE_MONO;
			break;
	}
	eeprom_write_byte((unsigned char *)COLORMODE_EEPROM_STORE, desklamp.colormode);
}

/**
 *  @brief      Set adapter
 *  @param    	isAdapter (0, 1)
*/
void desklamp_set_adapter(uint8_t isAdapter){
	desklamp.isAdapter = isAdapter ? 1 : 0;
	eeprom_write_byte((unsigned char *)ADAPTER_EEPROM_STORE, desklamp.isAdapter);
}

void desklamp_set_serial(uint32_t serial) {
	desklamp.serial = serial;
	eeprom_write_block(&desklamp.serial, (unsigned char *)SERIAL_EEPROM_STORE, 4);
}

/**
 *  @brief      Set desklamp strobe
 *  @param    	strobe (0..255)
*/
void desklamp_set_strobe(uint8_t strobe){
		desklamp.strobe = strobe;
}

/**
 *  @brief      Set desklamp blackout
 *  @param    	blackout (0..1)
*/
void desklamp_set_blackout(uint8_t blackout) {
		desklamp.blackout = blackout;
}

/**
 *  @brief      Set current desklamp state
 *  @param    	state (compare desklamp.h)
*/
void desklamp_set_state(uint8_t state){
	desklamp.state = state;
}


/** GET Functions */

/**
 *  @brief      Get current strobe value
 *
 *  experimental
 *  @return		strobe (Hz)
*/
uint8_t desklamp_get_strobe(void){
	return desklamp.strobe;
}


/**
 *  @brief      Get current desklamp state
 *  @return		state (compare desklamp.h)
*/
uint8_t desklamp_get_state(void){
	return desklamp.state;
}

/**
 *  @brief      Get current colormode
 *  @return		colormode (RGB, MONO)
*/
uint8_t desklamp_get_colormode(void){
	return desklamp.colormode;
}

/**
 *  @brief      Is this DeskLamp an adapter?
 *  @return		1 = adapter, 0 = directly connected LEDs
*/
uint8_t desklamp_is_adapter(void){
	return desklamp.isAdapter;
}

/**
 *  @brief      Get serial
 *  @return		serial
*/
uint32_t desklamp_get_serial(void){
	return desklamp.serial;
}

/**
 *  @brief      Get current dimmer value
 *  @return		value
*/
uint8_t desklamp_get_dimmer(void){
	return desklamp.dimmer;
}

/**
 *  @brief      Get current RGB Value
 *  @param    	c ('r', 'g', 'b')
 *  @return		value
*/
uint8_t desklamp_get_rgb(char c){
	uint8_t return_val  = 0;

	switch (c) {
		case 'r':
			return_val =  desklamp.r;
			break;
		case 'g':
			return_val =  desklamp.g;
			break;
		case 'b':
			return_val =  desklamp.b;
			break;
	}
	return return_val;
}


/* General Functions */

/**
 *  @brief      Update current PWM Values
*/
void desklamp_update_pwm(void){
	uint16_t dimmer = desklamp.dimmer;
	if (desklamp.blackout) {
		dimmer = 0;
	}
	if (desklamp.colormode == DESKLAMP_COLORMODE_RGB) {
		uint8_t red_pwm = pgm_read_byte(&(pwmtable[(((uint16_t)desklamp.r * dimmer) >> 9)]));
		if (red_pwm == 0) {
			desklamp_set_led(1, OFF);
			desklamp_config_channel(1, DISABLE);
		} else {
			OCR0B = red_pwm;
			desklamp_config_channel(1, ENABLE);
		}
		OCR1A = pgm_read_byte(&(pwmtable[(((uint16_t)desklamp.g * dimmer) >> 9)]));
		OCR1B = pgm_read_byte(&(pwmtable[(((uint16_t)desklamp.b * dimmer) >> 9)]));
	} else {	// COLORMODE_MONO
		uint8_t pwm = pgm_read_byte(&(pwmtable[((uint8_t)dimmer >> 1)]));
		if (pwm == 0) {
			desklamp_set_led(1, OFF);
			desklamp_config_channel(1, DISABLE);
		} else {
			OCR0B = pwm;
			desklamp_config_channel(1, ENABLE);
		}
	}
}

/**
 *  @brief      Check external USB Device
 *
 *  This function checks, if an external USB device is connected
 *  @return		0= No Device; 1= Ext. Device
*/
uint8_t desklamp_chk_extusb(void){
	// check USB_EXT
	if ((PINA & (1 << DESKLAMP_PIN_DP_EXT)) || (PINA & (1 << DESKLAMP_PIN_DM_EXT))) {
		desklamp.usb_ext = 1;
		return 1;
	} else {
		desklamp.usb_ext = 0;
		return 0;
	}
}

/* --------------------------------- End Of File ------------------------------ */
