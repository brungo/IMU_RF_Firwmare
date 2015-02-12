/*
 * PlacaSensores.c
 *
 * Created: 07/09/2014 09:52:00 p.m.
 *  Author: Bruno
 */ 
#define __ATMEGA644RFR2__
#define F_CPU	8000000UL
#define F_TWI   100000UL
#define FOSC	8000000
#define BAUD	9600
#define MY_UBRR	(FOSC/16/BAUD-1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MPULib/i2c.h"
#include "MPULib/rbuff.h"
#include "MPULib/MPU9150.h"
#include <util/twi.h>
/*----------------------------------------------------------------------
PARTICULAR DEFINES and extern methods/functions includes
------------------------------------------------------------------------*/
//#define F_CLK	16000000
extern int mpu_select_device(int device);


/*----------------------------------------------------------------------
Global Variables definitions
------------------------------------------------------------------------*/
uint8_t buffer[256] = {0};
volatile rbuff_t * r_b, r_buffer_aux; 	
ing_buff_init (volatile rbuff_t *const rb, uint8_t *const ring);	


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

void init_serial(unsigned int ubrr)				//TODO implementar para el resto de las USART
{
	UBRR0H = (unsigned char) ubrr >> 8;
	UBRR0L = (unsigned char) ubrr;
	UCSR0B = 1 << TXEN0 | 1 << RXEN0;			// Rx and Tx enable
	UCSR0C = 1 << UPM00 | 1 << UPM01 | 1 << UCSZ00 | 1 << UCSZ01; // 8 data bits, paridad par, 1 stop bit
};

void init_periph(void)
{
	inittimer();
	init_i2c();
	init_serial(MY_UBRR);
	mpu_select_device(0);
	DDRB |= (1<<DDRB4);         // set LED pin PB4 to output
}

int main(void)
{

    init_periph();
    while(1)
    {
		PORTB |= 0x10;   // toggle pinb4
		_delay_ms(100);         // delay 100 ms
		PORTB = 0x00;   // toggle pb4
		_delay_ms(400);         // delay 400 ms		
	}
}
