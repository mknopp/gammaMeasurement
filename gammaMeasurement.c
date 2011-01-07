/* gammaMeasurement.c
 *
 * This program is meant to be run on an Atmel AVR microcontroller and
 * provides an UART interface to the 'Monitor 414' gamma dose measurement
 * unit by Genitron.
 *
 * The second task is the measurement of absolute air pressure using a
 * MPX4115A sensor by Freescale Semiconductors which is also included in
 * the output on the UART. Note that the measured air pressure is not altitude
 * compensated and needs to be done on the host computer (no need to reflash the
 * microcontroller when changing location)
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
#include <util/delay.h>

#include <stdio.h>

#include "USI_UART.h"

/* Function definitions */
void ADC_Init(void);
uint16_t ADC_Read(void);
static int uart_putchar(char c, FILE *stream);
int main(void) __attribute__((OS_main));

/* Small wrapper function to allow printf and friends */
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static int uart_putchar(char c, FILE *stream) {
	USI_UART_Transmit_Byte(c);

	return 0;
}

int main(void) {
	USI_UART_Flush_Buffers();
	ADC_Init();

	stdout = &mystdout;

	while(1) {
		uint8_t hByte, lByte;
		ADCSR |= (1 << ADSC); // Starte Konvertierung

		while (ADCSR & (1 << ADSC)); // Warte auf Konvertierung

		lByte = ADCL;
		hByte = ADCH;

		printf("Hallo, Welt\n");

/*		USI_UART_Transmit_Byte(hByte);
		USI_UART_Transmit_Byte(lByte);
		for (int i=0; i<5; ++i)
			USI_UART_Transmit_Byte(0);
*/
		_delay_ms(2000);
	}
}

void ADC_Init() {
	uint16_t result;

	ADMUX = 0; // Referenz: AVCC, right-aligned, ADC0
	ADCSR = (1 << ADEN) | (1 << ADSC) | (0 << ADFR) | (0 << ADIE);

	while (ADCSR & (1 << ADSC)); // warte erste Konvertierung ab

	result = ADCL;
	result = result << 8;
	result |= ADCH;
}

uint16_t ADC_Read() {
	uint16_t result;

	ADCSR |= (1 << ADSC); // Starte Konvertierung

	while (ADCSR & (1 << ADSC)); // Warte auf Konvertierung

	result = ADCL;
	result = result << 8;
	result |= ADCH;

	return result;
}
