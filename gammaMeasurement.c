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
		uint8_t spare7 :1,
				spare6 :1,
				spare5 :1,
				spare4 :1,
				spare3 :1,
				spare2 :1,
				spare1 :1,
				spare0 :1;
	};
} StatusFlags;

volatile uint16_t gammaDose = 0;


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
	char numberBuffer[5];

	// Enable Pin Change Interrupt on Port A, disable Port B IRQ and INT0
	GIMSK = PCIE1;

	USI_UART_Flush_Buffers();
	ADC_Init();

	while (1) {
		uint16_t airPressure;

		airPressure = ADC_Read();
		print(utoa(airPressure, numberBuffer, 10));
		print("\n");

		_delay_ms(2000);
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
	uint16_t result, temp;

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

// Interrupt handler for signals from Monitor 414 unit (count, save, alarm)
ISR(IO_PINS_vect) {
	static StatusFlags status = {0};


}
