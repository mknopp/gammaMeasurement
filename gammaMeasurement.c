/* gammaMeasurement.c
 *
 * This program is meant to be run on an Atmel AVR microcontroller and
 * provides an UART interface to the 'Monitor 414' gamma dose measurement
 * unit by Genitron.
 *
 * The second task is the measurement of absolute air pressure using a
 * MPX4115A sensor by Freescale Semiconductors which is also included in
 * the output on the UART. Note that the measured air pressure is not altitude
 * compensated and needs to be recalculated on the host computer (no need to
 * reflash the microcontroller when changing location)
 *
 * Copyright 2011 Martin Knopp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "USI_UART.h"

/**** Variables ****/
// Prototype for Monitor 414 status bits
typedef union {
	uint8_t allFlags;

	struct {
		uint8_t prevCount :1,
				prevAlarm :1,
				spare5 :1,
				spare4 :1,
				spare3 :1,
				spare2 :1,
				spare1 :1,
				spare0 :1;
	};
} StatusFlags;

volatile uint16_t gammaDose = 0;
volatile uint8_t transmitData = 0;
volatile uint8_t alarm = 0;


/**** Function definitions ****/
//! Initialize a/d converter
void ADC_Init(void);
//! Make one 12 bit a/d conversion and return result
uint16_t ADC_Read(void);
// OS_main saves 30 bytes, because main() will never return
int main(void) __attribute__((OS_main));
//! Send \param text on serial interface
void print(char *text);


/**** Actual program ****/
int main(void) {
	// Disable analog comparator, we need PA6 and PA7 for IRQs from the Monitor unit
	ACSR = (1<<ACD);
	// Enable pull-up on all non-analog-pins
	PORTA = (1<<PA7)|(1<<PA6)|(1<<PA5)|(1<<PA4)|(0<<PA3)|(1<<PA2)|(0<<PA1)|(0<<PA0);
	// Enable pull-up on all pins
	PORTB = (1<<PB7)|(1<<PB6)|(1<<PB5)|(1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);

	// Configure Interrupt 0 for rising edge
	MCUCR |= (1<<ISC00)|(1<<ISC01);
	// Setup interrupt mask: Enable INT0 and Pin Change on port A, disable Pin Change on port B
	GIMSK = (1<<PCIE1)|(1<<INT0);

	USI_UART_Flush_Buffers();
	ADC_Init();

	print("Starting up...\n");

	// Enable interrupts
	sei();

	while (1) {
		if (transmitData) {
			// Disable Pin Change IRQ during UART transmission
			cli();
			GIMSK &= ~((1<<PCIE1)|(1<<INT0));
			sei();

			char numberBuffer[11];

			print(utoa(gammaDose, numberBuffer, 10));
			gammaDose = 0;
			print(";");

			// Average over 512 readings and treat result as 12bit
			uint32_t airPressure = 0;
			for (uint16_t i=0; i<512; ++i) {
				airPressure += ADC_Read();
			}
			airPressure = airPressure >> 7;

			print(ultoa(airPressure, numberBuffer, 10));
			print(";");

			print(utoa(alarm, numberBuffer, 10));
			alarm = 0;
			print("\r\n");

			transmitData = 0;

			// Reenable Pin Change IRQ
			cli();
			GIMSK |= (1<<PCIE1)|(1<<INT0);
			sei();
		}
	}
}

void ADC_Init() {
	ADMUX = (1 << REFS1) | // Ref: int 2.56V
			(1 << REFS0) | // AREF 100nF to GND
			(0 << ADLAR) | // right adjusted
			(0 << MUX4) | // Diff: ADC0 - ADC1, 1x gain
			(1 << MUX3) |
			(1 << MUX2) |
			(0 << MUX1) |
			(0 << MUX0);
	ADCSR = (1 << ADEN) | // Enable ADC
			(1 << ADSC) | // Start conversion
			(0 << ADFR) | // Disable free running mode, only convert on demand
			(0 << ADIE) | // Disable ADC interrupt
			(1 << ADPS2) | // Set prescaler to 128
			(1 << ADPS1) | // -> 115.2 kHz sample rate
			(1 << ADPS0);  // Should be between 50 and 200 kHz

	while (ADCSR & (1 << ADSC)); // Wait for first conversion
}

uint16_t ADC_Read() {
	ADCSR |= (1 << ADSC); // Start conversion
	while (ADCSR & (1 << ADSC)); // Wait for conversion
	return ADC;
}

void print(char *text) {
	char *temp = text;

	while (*temp) {
		USI_UART_Transmit_Byte(*temp);
		++temp;
	}
}

// Interrupt handler for count and alarm signals from Monitor 414 unit
ISR(IO_PINS_vect) {
	static StatusFlags status = {0};

	uint8_t currCount = (PINA & (1<<PINA6)) >> PINA6;
	uint8_t currAlarm = (PINA & (1<<PINA7)) >> PINA7;

	// Boolean status flags for flank detection
	uint8_t countRise = 0;
	uint8_t alarmRise = 0;

	if (!status.prevCount && currCount)
		countRise = 1;
	if (!status.prevAlarm && currAlarm)
		alarmRise = 1;

	if (countRise) {
		++gammaDose;
	}

	// Just count the alarms as we only do logging
	if (alarmRise) {
		++alarm;
	}


	status.prevCount = currCount;
	status.prevAlarm = currAlarm;
}

// Interrupt handler for save signal from Monitor 414 unit
ISR(INT0_vect) {
	transmitData = 1;
}
