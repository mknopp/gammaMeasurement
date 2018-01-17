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
	// Disable Analog Comparator
//	ACSR = (1<<ACD);

	// Reset all ports to unused state (input, pull-up enabled)
	PORTA = (1<<PA7)|(1<<PA6)|(1<<PA5)|(1<<PA4)|(1<<PA3)|(1<<PA2)|(1<<PA1)|(1<<PA0);
	PORTB = (1<<PB7)|(1<<PB6)|(1<<PB5)|(1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);
	DDRA = (1<<DDA1) | (1<<DDA4) | (1<<DDA5) | (1<<DDA6);

	// Configure Interrupt 0 for rising edge
	MCUCR |= (1<<ISC00)|(1<<ISC01);
	// Setup interrupt mask: Enable INT0 and Pin Change on port A, disable Pin Change on port B
	GIMSK = (1<<PCIE1)|(1<<INT0);

	USI_UART_Flush_Buffers();
	ADC_Init();

	// Enable interrupts
	sei();

	print("Starting up...\n");

	uint8_t state = 0;

	while (1) {
		if (transmitData) {
			// Disable Pin Change IRQ during UART transmission
			cli();
			GIMSK &= ~((1<<PCIE1)|(1<<INT0));
			sei();

			char numberBuffer[6];

			print(utoa(gammaDose, numberBuffer, 10));
			gammaDose = 0;
			print(";");

			uint16_t airPressure = ADC_Read();
			print(utoa(airPressure, numberBuffer, 10));
			print("\n");

			transmitData = 0;
			PORTA &= ~(1<<PA4);

			// Reenable Pin Change IRQ
			cli();
			GIMSK |= (1<<PCIE1)|(1<<INT0);
			sei();
		}

		if (state) {
			PORTA &= ~(1<<PA5);
			state = 0;
		}
		else {
			PORTA |= (1<<PA5);
			state = 1;
		}

	}
}

void ADC_Init() {
	volatile uint8_t temp;

	ADMUX = 0; // Referenz: AVCC, right-aligned, ADC0
	ADCSR = (1 << ADEN) | (1 << ADSC) | (0 << ADFR) | (0 << ADIE);

	while (ADCSR & (1 << ADSC))
		; // warte erste Konvertierung ab

	temp = ADCL;
	temp = ADCH;
}

uint16_t ADC_Read() {
	uint8_t temp;
	uint16_t result;

	ADCSR |= (1 << ADSC); // Starte Konvertierung

	while (ADCSR & (1 << ADSC))
		; // Warte auf Konvertierung

	temp = ADCL;
	result = ADCH;
	result = result << 8;
	result |= temp;
	return result;
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

	uint8_t currCount = (PINA & (1<<PINA3)) >> PINA3;
//	uint8_t currAlarm = (PINA & (1<<PINA7)) >> PINA7;

	// Boolean status flags for flank detection
	uint8_t countRise = 0;
//	uint8_t alarmRise = 0;

	if (!status.prevCount && currCount)
		countRise = 1;
/*	if (!status.prevAlarm && currAlarm)
		alarmRise = 1;
*/
	if (countRise) {
		PORTA |= (1<<PA1);
		++gammaDose;
	}
	else
		PORTA &= ~(1<<PA1);

//	if (alarmRise)
		// TODO alarm?


	status.prevCount = currCount;
//	status.prevAlarm = currAlarm;
}

// Interrupt handler for save signal from Monitor 414 unit
ISR(INT0_vect) {
	PORTA |= (1<<PA4);
	transmitData = 1;
}
