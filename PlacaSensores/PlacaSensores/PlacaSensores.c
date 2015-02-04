/*
 * PlacaSensores.c
 *
 * Created: 07/09/2014 09:52:00 p.m.
 *  Author: Bruno
 */ 
#define __ATMEGA644RFR2__
#define F_CPU	8000000UL
#define F_TWI   100000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "rbuff.h"
#include "MPU9150.h"
#include <util/twi.h>
/*----------------------------------------------------------------------
DEFINES
------------------------------------------------------------------------*/
//#define F_CLK	16000000

void inittimer(void)
{
	//TIMER 0
	// Descripción de registros  
	TCCR0A=(0<<WGM00) | (0<<WGM01) | (0<<WGM02) ; //Normal mode, prescaler clk/8.
    TCCR0B= (0<<CS02) |  (1<<CS01) |  (0<<CS00);
}
void init_i2c()
{
	i2c_init_(0x45);
}

int main(void)
{
	inittimer();
	init_i2c();

	DDRB |= (1<<DDRB4);          // set LED pin PB4 to output
    while(1)
    {
		PORTB |= 0x10;   // toggle pinb4
		_delay_ms(1);         // delay 1 ms
		PORTB = 0x00;   // toggle pb4
		_delay_ms(9);         // delay 9 ms
		
	}
}
