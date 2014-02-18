/*
 * entropy.c
 *
 *  Created on: 13.02.2014
 *      Author: Stefan Krupop
 */

#include "entropy.h"
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define WDT_BUFFER_SIZE	32

volatile uint8_t gWDT_pool_count;
volatile uint32_t gWDT_entropy_pool;
uint8_t gWDT_buffer_position;

void entropy_init(void) {
	gWDT_buffer_position = 0;
	gWDT_pool_count = 0;
	cli(); // Temporarily turn off interrupts, until WDT configured
	MCUSR = 0; // Use the MCU status register to reset flags for WDR, BOR, EXTR, and POWR
	_WD_CONTROL_REG |= (1 << _WD_CHANGE_BIT) | (1 << WDE);
	// WDTCSR |= _BV(WDCE) | _BV(WDE);// WDT control register, This sets the Watchdog Change Enable (WDCE) flag, which is  needed to set the
	_WD_CONTROL_REG = _BV(WDIE); // Watchdog system reset (WDE) enable and the Watchdog interrupt enable (WDIE)
	TCCR1B = (1 << CS10);					// prescaler 1
	sei(); // Turn interupts on
}

uint32_t entropy_random(void) {
	while (gWDT_pool_count < 1);

	gWDT_pool_count = 0;
	return gWDT_entropy_pool;
}

// This interrupt service routine is called every time the WDT interrupt is triggered.
// With the default configuration that is approximately once every 16ms, producing
// approximately two 32-bit integer values every second.
//
// The pool is implemented as an 8 value circular buffer
ISR(WDT_vect) {
	static uint8_t gWDT_buffer[WDT_BUFFER_SIZE];

	gWDT_buffer[gWDT_buffer_position] = TCNT1L; // Record the Timer 1 low byte (only one needed)
	gWDT_buffer_position++; // every time the WDT interrupt is triggered
	if (gWDT_buffer_position >= WDT_BUFFER_SIZE) {
		// The following code is an implementation of Jenkin's one at a time hash
		// This hash function has had preliminary testing to verify that it
		// produces reasonably uniform random results when using WDT jitter
		// on a variety of Arduino platforms
		for (uint8_t i = 0; i < WDT_BUFFER_SIZE; ++i) {
			gWDT_entropy_pool += gWDT_buffer[i];
			gWDT_entropy_pool += (gWDT_entropy_pool << 10);
			gWDT_entropy_pool ^= (gWDT_entropy_pool >> 6);
		}
		gWDT_entropy_pool += (gWDT_entropy_pool << 3);
		gWDT_entropy_pool ^= (gWDT_entropy_pool >> 11);
		gWDT_entropy_pool += (gWDT_entropy_pool << 15);
		gWDT_buffer_position = 0; // Start collecting the next 32 bytes of Timer 1 counts
		gWDT_pool_count = 1;
	}
}
