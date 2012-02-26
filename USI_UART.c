/* USI_UART.c
 *
 * This file is a combined version of the original application note and
 * modifications for avr-gcc taken from http://www.mikrocontroller.net/topic/57023
 *
 * All code for reception was removed to free up interrupt routines and flash
 *
 * Martin Knopp, 2011
 *
 */

/*****************************************************************************
*
* Copyright (C) 2003 Atmel Corporation
*
* File              : USI_UART.c
* Compiler          : IAR EWAAVR 2.28a
* Created           : 18.07.2002 by JLL
* Modified          : 02-10-2003 by LTA
*
* Support mail      : avr@atmel.com
*
* Supported devices : ATtiny26
*
* Application Note  : AVR307 - Half duplex UART using the USI Interface
*
* Description       : Functions for USI_UART_receiver and USI_UART_transmitter.
*                     Uses Pin Change Interrupt to detect incoming signals.
*
*
****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "USI_UART.h"

//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))

#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

//********** Static Variables **********//
register unsigned char USI_UART_TxData asm("r3");				// Tells the compiler to use Register 15 instead of SRAM

static unsigned char          UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

static volatile union USI_UART_status                           // Status byte holding flags.
{
    unsigned char status;
    struct
    {
        unsigned char ongoing_Transmission_From_Buffer:1;
        unsigned char ongoing_Transmission_Of_Package:1;
        unsigned char reception_Buffer_Overflow:1;
        unsigned char flag4:1;
        unsigned char flag5:1;
        unsigned char flag6:1;
        unsigned char flag7:1;
    };
} USI_UART_status = {0};


//********** USI_UART functions **********//

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x )
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

// Flush the UART buffers.
void USI_UART_Flush_Buffers( void )  
{  
    UART_TxTail = 0;
    UART_TxHead = 0;
}

// Initialize USI for UART transmission.
void USI_UART_Initialise_Transmitter( void )                              
{
    cli();
    TCNT0  = 0x00;	//Zähler0 auf 0 setzen
    TCCR0  = (1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);         // Reset the prescaler and start Timer0.
    TIFR   = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag.
    TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.
                                                                
    USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
             (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
             (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source.
             (0<<USITC);                                           
             
    USIDR  = 0xFF;                                            // Make sure MSB is '1' before enabling USI_DO.
    USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
             0x0F;                                            // Preload the USI counter to generate interrupt at first USI clock.
    DDRB  |= (1<<PB1);                                        // Configure USI_DO as output.
                  
    USI_UART_status.ongoing_Transmission_From_Buffer = TRUE;
                  
    sei();
}

// Puts data in the transmission buffer, after reversing the bits in the byte.
// Initiates the transmission routines if not already started.
void USI_UART_Transmit_Byte( unsigned char data )          
{
    unsigned char tmphead;

    tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;        // Calculate buffer index.
    while ( tmphead == UART_TxTail );                           // Wait for free space in buffer.
    UART_TxBuf[tmphead] = Bit_Reverse(data);                    // Reverse the order of the bits in the data byte and store data in buffer.
    UART_TxHead = tmphead;                                      // Store new index.
    
    if ( !USI_UART_status.ongoing_Transmission_From_Buffer )    // Start transmission from buffer (if not already started).
        USI_UART_Initialise_Transmitter();              
}

// ********** Interrupt Handlers ********** //

// The USI Counter Overflow interrupt is used for moving data between memory and the USI data register.
// The interrupt is used for both transmission and reception.
ISR(SIG_USI_OVERFLOW)
{
    unsigned char tmptail;
    
    // Check if we are running in Transmit mode.
    if( USI_UART_status.ongoing_Transmission_From_Buffer )      
    {
        // If ongoing transmission, then send second half of transmit data.
        if( USI_UART_status.ongoing_Transmission_Of_Package )   
        {                                   
            USI_UART_status.ongoing_Transmission_Of_Package = FALSE;    // Clear on-going package transmission flag.
            
            USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
            USIDR = (USI_UART_TxData << 3) | 0x07;                      // Reload the USIDR with the rest of the data and a stop-bit.
        }
        // Else start sending more data or leave transmit mode.
        else
        {
            // If there is data in the transmit buffer, then send first half of data.
            if ( UART_TxHead != UART_TxTail )                           
            {
                USI_UART_status.ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.
                
                tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;    // Calculate buffer index.
                UART_TxTail = tmptail;                                  // Store new index.            
                USI_UART_TxData = UART_TxBuf[tmptail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
                                                                        // The bit reversing is moved to the application section to save time within the interrupt.
                USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
                USIDR  = (USI_UART_TxData >> 2) | 0x80;                 // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
                                                                        //  of bit of bit reversed data).                
            }
            // Else enter receive mode.
            else
            {
                USI_UART_status.ongoing_Transmission_From_Buffer = FALSE; 
                
                TCCR0  = (0<<CS02)|(0<<CS01)|(0<<CS00);                 // Stop Timer0.
                PORTB |=   (1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);         // Enable pull up on USI DO, DI and SCK pins. (And PB3 because of pin change interrupt)   
                DDRB  &= ~((1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0));        // Set USI DI, DO and SCK pins as inputs.  
                USICR  =  0;                                            // Disable USI.
                GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
                GIMSK |=  (1<<PCIE0);                                   // Enable pin change interrupt for PB3:0.
            }
        }
    }
}

// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
ISR(SIG_OVERFLOW0)
{
    TCNT0 += TIMER0_SEED;                   // Reload the timer,
                                            // current count is added for timing correction.
}
