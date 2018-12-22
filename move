/*
 * Crow-Robot.c
 *
 * Created: 13-12-2018 22:26:14
 * Author : ERTS 1
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void magnet_pin_config()
{
	DDRH = 0x01;
	PORTH = 0x00;
}

void motor_pin_config()
{
	DDRA = 0x0F;
	PORTA = 0x00;
}

void magnet_on()
{
	PORTH = 0x01;
}

void magnet_off()
{
	PORTH = 0x00;
}

void forward()
{
	PORTA = 0x05;
}

void backward()
{
	PORTA = 0x0A;
}



void stop()
{
	PORTA = 0x00;
}




int main(void)
{
    /* Replace with your application code */
	motor_pin_config();
	magnet_pin_config();
    while (1) 
    {
		forward();
		_delay_ms(3000);
		stop();
		magnet_on();
		_delay_ms(3000);
		backward();
		_delay_ms(3000);
		magnet_off();
		stop();
		_delay_ms(3000);
    }
}

