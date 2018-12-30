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
	DDRH = ;
	PORTH = ;
}

void motor_pin_config()
{
	DDRA = ;
	PORTA = ;
}

void magnet_on()
{
	PORTH = ;
}

void magnet_off()
{
	PORTH = ;
}

void forward()
{
	PORTA = ;
}

void backward()
{
	PORTA = ;
}

void left()
{
	PORTA = ;
}

void right()
{
	PORTA = ;
}

void soft_left()
{
	PORTA = ;
}

void soft_right()
{
	PORTA = ;
}

void stop()
{
	PORTA = ;
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

