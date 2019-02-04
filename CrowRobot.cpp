/*
 * Crow-Robot.c
 *
 * Created: 13-12-2018 22:26:14
 * Author : ERTS 1
 */ 

/*
* Team Id: 972
* Author List: Harshita Didee, Shivam Grover, Shivani Jindal, Anuj Trehan
* Filename: main.c
* Theme: Thirsty-Crow
* Functions: magnet_pin_config(), magnet_pin_config(), timer5_init(), velocity(), magnet_on(), magnet_off(), forward(),	backward(), soft_left(), soft_right(), stop(), main()
* Global Variables: none
*/


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>	

unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char ADC_Value;

#define RX  (1<<4)
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)
volatile unsigned char data;

/*
 * Function Name: uart0_init()
 * Input: none
 * Output: none
 * Logic: It initializes the registers for xbee communication
 * Example Call: uart0_init();
 */

void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; 							//9600BPS at 14745600Hz
	UBRR0H = 0x00;
	UCSR0B = 0x98;
	//UCSR0C = 3<<1;						//setting 8-bit character and 1 stop bit
	//UCSR0B = RX | TX;
}

/*
 * Function Name: uart_tx(char data)
 * Input: char data
 * Output: none
 * Logic: this function is used to transmit data to the python console through xBee communication
 * Example Call: uart_tx(data);
 */

void uart_tx(char data)
{
	while(!(UCSR0A & TE));						//waiting to transmit
	UDR0 = data;
}



ISR(USART0_RX_vect)  		//interrupt
{
	data = UDR0;
}

/*
 * Function Name: uart_rx()
 * Input: none
 * Output: none
 * Logic: this function is used to receive data from the python console through xBee communication
 * Example Call: data = uart_rx();
 */

char uart_rx()
{
	while(!(UCSR0A & RE));						//waiting to receive
	return UDR0;
}


/*
? * Function Name: magnet_pin_config
? * Input: none
? * Output: none
? * Logic: Initializes the pins of port H for the use of electromagnet
? * Example Call: magnet_pin_config();
*/


void magnet_pin_config()
{
	DDRH = 0x01;	//setting pin 1 of port H as output
	PORTH = 0x00;	//setting PH1 initially as logic 0
}
void buzzer_pin_config()
{
	DDRB = 0x01;	//setting pin 1 of port H as output
	PORTB = 0x00;	//setting PH1 initially as logic 0
}


/*
? * Function Name: motor_pin_config
? * Input: none
? * Output: none
? * Logic: Initializes the pins of port A for use by the motors. Also calls timer5_init to set the timer for PWM use
? * Example Call: motor_pin_config();
*/

void motor_pin_config()
{
	DDRA = 0x0F;	//setting pins 0,1,2,3 of port A as output
	PORTA = 0x00;	//setting all pins of port A initially as logic 0
	DDRL =  0x18;   //Setting pin 3 of port L and pin 4 of port L as output for PWM generation
	PORTL = 0x18;	//PL3 and PL4 pins are for velocity control using PWM.
	timer5_init();	// since we can't change the main function we called this here to enable timer
}

/*
? * Function Name: magnet_pin_config
? * Input: none
? * Output: none
? * Logic: Initializes timer 5 in PWM mode for velocity control
? * Example Call: timer5_init();
*/
void timer5_init()
{
	TCCR5B = 0x00;		//Stop
	TCNT5H = 0xFF;		//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;		//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;		//Output compare register high value for Left Motor
	OCR5AL = 0xFF;		//Output compare register low value for Left Motor
	OCR5BH = 0x00;		//Output compare register high value for Right Motor
	OCR5BL = 0xFF;		//Output compare register low value for Right Motor
	OCR5CH = 0x00;		//Output compare register high value for Motor C1
	OCR5CL = 0xFF;		//Output compare register low value for Motor C1
	TCCR5A = 0xA9;		
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
? * Function Name: velocity
? * Input: left_motor -> velocity of left motor(from 0 to 255)
		   right_motor -> velocity of right motor(from 0 to 255) 
? * Output: none
? * Logic: Sets the velocity of the motors
? * Example Call: velocity(255,250);
*/

void velocity (unsigned char left_motor, unsigned char right_motor)		
{
	float i = 0.75;
	OCR5AL = (unsigned char)right_motor*i;		//set the velocity for left motor or motor 1
	OCR5BL = (unsigned char)left_motor*i;	        //set the velocity for right motor or motor 2
}

/*
? * Function Name: magnet_on
? * Input: none
? * Output: none
? * Logic: Turns on the electromagnet. By sending logic high to pin 0 of port H
? * Example Call: magnet_on();
*/

void magnet_on()	
{
	PORTH = 0x01;	//setting pin 0 of port H as logic 1
}

void buzzer_on()
{
	PORTB = 0x01;	//setting pin 0 of port H as logic 1
}

/*
? * Function Name: magnet_off
? * Input: none
? * Output: none
? * Logic: Turns off the electromagnet. By sending logic low to pin 0 of port H
? * Example Call: magnet_off();
*/

void magnet_off()	
{
	PORTH = 0x00;	//setting all pins of port H as logic 0
}

void buzzer_off()
{
	PORTB = 0x00;	//setting all pins of port H as logic 0
}

/*
? * Function Name: forward
? * Input: none
? * Output: none
? * Logic: moves the robot forward in a straight line by setting pins 2 and 0 of port A as logic 1
? * Example Call: forward();
*/

void forward()		
{
	velocity(255,245);	/*at maximum values of velocity for both motors, the robot would move a bit leftwards. So to make it go linear we take the velocity of the left motor as 255 and of the right motor as 225*/
	
	PORTA = 0x05;		//setting pins 2 and 0 of port A as logic 1
}

/*
? * Function Name: backward
? * Input: none
? * Output: none
? * Logic: moves the robot backwards in a straight line by setting pins 3 and 1 of port A as logic 1
? * Example Call: backward();
*/

void backward()
{
	velocity(255,240);	//setting velocity of left motor as 255 and right motor a 240
						//as a straight backwards motion was observed at these velocities 
	
	PORTA = 0x0A;		//setting pins 3 and 1 of port A as logic 1	
}

/*
? * Function Name: left
? * Input: none
? * Output: none
? * Logic: rotates the robot in the left direction by making the right motor rotate in forward direction and the left motor rotate backward direction by setting pins 3 and 0 of port A as logic 1
? * Example Call: left();
*/

void left()
{
	velocity(255,245);
	PORTA =	0x09;		//setting pins 3 and 0 of port A as logic 1
}

/*
? * Function Name: right
? * Input: none
? * Output: none
? * Logic: rotates the robot in the right direction by making the left motor rotate in forward direction and the right motor rotate backward direction by setting pins 2 and 1 of port A as logic 1
? * Example Call: right();
*/

void right()
{
	velocity(255,240);
	PORTA = 0x06;		//setting pins 2 and 1 of Port A as logic 1
}

/*
? * Function Name: soft_left
? * Input: none
? * Output: none
? * Logic: making a left turn by making the right motor rotate in forward direction and keeping the left motor stopped by setting pin 2 port A as logic 1
? * Example Call: soft_left();
*/

void soft_left()
{
	velocity(220,220);
	PORTA = 0x02;		//setting pin 2 of Port A as logic 1
}

/*
? * Function Name: soft_right
? * Input: none
? * Output: none
? * Logic: making a right turn by making the left motor rotate in forward direction and keeping the right motor stopped by setting pin 1 port A as logic 1
? * Example Call: soft_right();
*/

void soft_right()
{
	velocity(220,220);
	PORTA = 0x08;		//setting pin 1 of port A as logic 1
}

/*
? * Function Name: stop
? * Input: none
? * Output: none
? * Logic: Stops the motors by setting all pins of port A as logic 1
? * Example Call: soft_left();
*/

void stop()
{
	velocity(255,0);	/*setting the velocity of the right motor as 0 before the left motor
						  since the right motor kept rotating for a few milliseconds longer than the left
						  motor if we stopped them at the same time, causing the robot to tilt in one direction resulting in a non linear traversal */
	_delay_ms(155);		//setting a delay of 155 milliseconds so that both motors stop rotating at the same time
	PORTA = 0x00;		//setting the pins of Port A as logic 0
	
}


/*
? * Function Name: main
? * Input: none
? * Output: none
? * Logic: performs the necessary actions given in this task
? * Example Call: none
*/

void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


int main(void)
{
	int j = 0;
	int ar[] = {0,0,0,0,0,0,1,1,1,1,1,1};
	char ins[20];
	buzzer_pin_config();

	uart0_init();
	int k =0;


	while(1){
		ins[k] = uart_rx();

		if(ins[k]=='d'){	//d is the final move where the pebble is dropped
		break;}
		k++;
		
	}
	
	cli();
	adc_pin_config();
	adc_init();
	timer5_init();
	sei();			int flag = 0;

    /* Replace with your application code */
	motor_pin_config();
	magnet_pin_config();

    while (1) 
    {
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			
			if (Center_white_line<100 && Right_white_line>100 && Left_white_line<100){
				flag = 1;
				
				soft_right();

				}
			if (Center_white_line<100 && Left_white_line>80 && Right_white_line<100){
				flag = 1;
				soft_left();

			}	
			if (Center_white_line>100 && Right_white_line<100 && Left_white_line<100){
				flag = 2;
				backward();
			}			
			if ((Center_white_line>120 && Right_white_line>120 && Left_white_line>120) || (Center_white_line>145 && Right_white_line<100 && Left_white_line>145) || (Center_white_line>145 && Right_white_line>145 && Left_white_line<100)){
				flag = 1;
				backward();
				_delay_ms(280);
				
				if(ins[j]=='l'){
				left();
				_delay_ms(270);
				}
				else if(ins[j]=='r'){
					right();
					_delay_ms(240);
				}
				else if(ins[j]=='s'){
					forward();
					_delay_ms(20);
					stop();
					buzzer_on();
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 's';
					_delay_ms(1000);
					buzzer_off();
					_delay_ms(1000);

					forward();
					_delay_ms(700);
					
					}
				else if(ins[j]=='d'){
					forward();
					_delay_ms(20);
					stop();
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'd';
					buzzer_on();
					_delay_ms(1000);
					buzzer_off();
					_delay_ms(1000);

					forward();
					_delay_ms(400);
					stop();
					break;
					
				}
				else if(ins[j]=='a'){
					left();
					_delay_ms(1100);
				}
				else if(ins[j]=='q'){
					right();
 					_delay_ms(600);
// 					backward();
// 					_delay_ms(100);
// 					stop();
// 					_delay_ms(3000);
					forward();
					_delay_ms(650);
					
				}
				else if(ins[j]=='w'){
					left();
					_delay_ms(800);
// 					backward();
// 					_delay_ms(100);
// 					stop();
// 					_delay_ms(3000);
					forward();
					_delay_ms(650);
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'a';
				}
				j++;
			}
			
			if (Center_white_line<30 && Right_white_line<30 && Left_white_line<30){
				stop();
			}
			
// 			if(Center_white_line>100)
// 			{
// 				flag=1;
// 				forward();
// 			}

// 			if((Left_white_line>20) && (flag==0))
// 			{
// 				flag=1;
// 				soft_right();
// 			}
// 
// 			if((Right_white_line>20) && (flag==0))
// 			{
// 				flag=1;
// 				soft_left();
// 			}
// 
// 			if((Center_white_line>100 && Left_white_line>100 && Right_white_line>100)||(Center_white_line>100&&Left_white_line>100)||(Center_white_line>100||Right_white_line>100)||(Center_white_line<50)&&(Left_white_line<50)&&(Right_white_line<40))
// 			{
// 				flag = 1;
// 				stop();
// 			}
			
			
			
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 			UDR0 = Left_white_line;
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 
// 			UDR0 = 'a';
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 
// 			UDR0 = Center_white_line;
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 
// 			UDR0 = 'a';
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 
// 			UDR0 = Right_white_line;
// 			while(!(UCSR0A & TE));			//waiting to transmit
// 
// 			UDR0 = 'a';
		//forward();
// 		_delay_ms(3000);
// 		stop();
// 		magnet_on();
// 		_delay_ms(3000);
// 		backward();
// 		_delay_ms(3000);
// 		magnet_off();
// 		stop();
// 		_delay_ms(3000);
    }
}
