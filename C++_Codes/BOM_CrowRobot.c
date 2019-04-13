/*/*
 * Crow-Robot.c
 *
 * Created: 13-12-2018 22:26:14
 * Author : ERTS 1
 */ 

/*
* Team Id: 972
* Author List: Harshita Diddee, Shivam Grover, Shivani Jindal, Anuj Trehan
* Filename: main.c
* Theme: Thirsty-Crow
* Functions: magnet_pin_config(), buzzer_pin_config(), motor_pin_config(), timer5_init(), 
			velocity(), magnet_on(), magnet_off(), buzzer_on(), buzzer_off(), forward(), 
			backward(), soft_left(), soft_right(), stop(),init(), servo_init, timer1_init(), servo_init(),
			servo_free(), servoaction(), main()
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
volatile unsigned char ins[50];
volatile int i =0; volatile int flag1 = 0, flag2 = 0,flaga = 0, flag = 10;
char c = 'c';

int thres=35; 
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



ISR(USART0_RX_vect)
{
	
	if(ins[i-1]!='f'){
		ins[i]= UDR0;
		///UDR0 = ins[i];
		if((ins[i])=='f'){
			UDR0 = 'N';
			flag1 = 1;
		}
		i++;
	}
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

/*
? * Function Name: buzzer_pin_config
? * Input: none
? * Output: none
? * Logic: Initializes the pins of port K for the use of electromagnet
? * Example Call: buzzer_pin_config();
*/


void buzzer_pin_config()
{
	DDRK = 0x01;	//setting pin 1 of port D as output
	PORTK = 0x00;	//setting PH1 initially as logic 0
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
? * Function Name: timer5_init
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
? * Function Name: timer1_init
? * Input: none
? * Output: none
? * Logic: Initializes the internal registers of timer1
? * Example Call: timer1_init();
*/

void timer1_init(){
	TCCR1A= 0x00;
	ICR1= 1023;

	TCNT1=0;
	OCR1A= 1023;
	OCR1B = 1023;
	OCR1C = 1023;
	TCCR1A= 0xAB;
	TCCR1B = 0x0C;
}

/*
? * Function Name: servo_init
? * Input: none
? * Output: none
? * Logic: Initializes the pins of port B for use by the servo.
? * Example Call: servo_init();
*/

void servo_init(){
	DDRB= DDRB | 0x20;
	PORTB= PORTB | 0x20;
	DDRB= DDRB | 0x40;
	PORTB= PORTB | 0x40;
}

/* Function name: servoleft_free()
   Input: void
   Output: void
   Logic: for making the servo free
   Example Call:servoleft_free()   */


void servo_free(void){
	_delay_ms(473);
	OCR1B= 1023;
}

/* Function name: servoaction()
   Input: void
   Output: void
   Logic: for moving the servo at particular angle to strike the key  
   Example Call:servoaction()  */
	
	void servoaction(unsigned char degrees){
	TCNT1=0;
	float regval2= ((float)(degrees*0.512) )+34.56;
	OCR1B= (uint16_t)regval2;
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
	float i = 1;
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

/*
? * Function Name: buzzer_on
? * Input: None
? * Output: None
? * Logic: Turns on the buzzer. By sending logic low to pin 0 of port K
? * Example Call: buzzer_on();
*/

void buzzer_on()
{
	PORTK = 0x01;	//setting pin 0 of port H as logic 1
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

/*
? * Function Name: buzzer_off
? * Input: None
? * Output: None
? * Logic: Turns off the buzzer. By sending logic low to pin 0 of port K
? * Example Call: buzzer_off();
*/

void buzzer_off()
{
	PORTK = 0x00;	//setting all pins of port H as logic 0
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
	velocity(255,243);	//setting velocity of left motor as 255 and right motor a 240
	//as a straight forwards motion was observed at these velocities
	
	PORTA = 0x0A;		//setting pins 3 and 1 of port A as logic 1
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
	velocity(255,250);		/*at maximum values of velocity for both motors, the robot would move a bit leftwards. So to make it go linear we take the velocity of the left motor as 255 and of the right motor as 225*/
	
	PORTA = 0x05;		//setting pins 2 and 0 of port A as logic 1
	
	
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
	velocity(255*0.7,245*0.7);
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
{	velocity(255*0.7,240*0.7);
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
	//velocity(220,220);
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
	//velocity(220,220);
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

	PORTA = 0x00;		//setting the pins of Port A as logic 0
	
}


/*
? * Function Name: adc_pin_config
? * Input: none
? * Output: none
? * Logic: Initializes the pins of port F for ADC use
? * Example Call: timer5_init();
*/

void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

/*
? * Function Name: adc_init
? * Input: none
? * Output: none
? * Logic: Initializes registers' values for ADC use
? * Example Call: timer5_init();
*/

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external		 ADLAR=1		 MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1	 ADIE=1		 ADPS2:0 = 1 1 0
}

/*
? * Function Name: ADC_Conversion
? * Input: Ch (unsigned char)
? * Output: a
? * Logic: function for adc conversion
? * Example Call: Right_white_line = ADC_Conversion(1)
*/

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

void init(){
	
	adc_init();
	timer1_init();
	servo_init();
	uart0_init();
	adc_pin_config();
	timer5_init();
	buzzer_pin_config();
	motor_pin_config();
	magnet_pin_config();

}
/*
? * Function Name: main
? * Input: none
? * Output: none
? * Logic: receives instructions from the python script as characters, performs line following, 
		   follows the instruction one by one at the nodes encountered, send triggers as characters when pebble is picked up and dropped
? * Example Call: none
*/
int main(void)
{
	
	/* Replace with your application code */
	cli();
	int j = 0;
	//char ins[50];
	int s = 0;
	init();
	sei();
	servoaction(15);
	servo_free();
	int k =0;
	

	while(!(UCSR0A & TE));			//waiting to transmit
	UDR0 = 'w';
	while(flag1	!=1);
	cli();	
	forward();
	{
	}
	
	
    while (1) 
    {
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor


			//when the black line is under the right led and not the others			
			if (Center_white_line<100 && Right_white_line>110 && Left_white_line<100){
				flag = 0;
				velocity(220,220);	
				soft_right();	
				}
			//when the black line is under the center led and not the others
			if (Center_white_line<100  && Left_white_line>120 && Right_white_line<100){
				flag = 1;
				velocity(220,220);
				soft_left();		
	

			}	
			//when the black line is under the center led and not the others
			if ((Center_white_line>100 && Right_white_line<100 && Left_white_line<100) /*|| (Center_white_line>50 && Right_white_line<50 && Left_white_line>50) || (Center_white_line>50 && Right_white_line>50 && Left_white_line<50)*/){
				flag = 2;
				//velocity(255,243);
				if(flag2==1){
					soft_left();
					_delay_ms(125);
					flag2 = 0;
				}
				if(flaga==1){
					soft_right();
					_delay_ms(125);
					flaga = 0;
				}
				forward();				

				
			}		
			
				
			//IF A NODE IS DETECTED
			if ((Center_white_line>120 && Right_white_line>120 && Left_white_line>120) || (Center_white_line + Left_white_line>240 ) || (Center_white_line + Right_white_line> 240)){
				//flag = 7;
				while(!(UCSR0A & TE));			//waiting to transmit
				UDR0 = ins[j];
				
				if(ins[j]=='l'){		//Turn left 30 degrees
					forward();
					_delay_ms(140);

				left();
				velocity(255*0.6,245*0.6);
				_delay_ms(200);
				flag2=1;
				
				}
				else if(ins[j]=='r'){		//Turn right 30 degrees
					forward();
					_delay_ms(140);

					right();
					velocity(255*0.6,240*0.6); 
					_delay_ms(200);
					flaga = 1;
					
				}
				else if(ins[j]=='s'){		//picking up pebble
					backward();				//one of our motors used to stop later than the second motor and would jerk sideways
											//to overcome this we gave it opposite polarity for small interval of time
											//this solved the problem just fine
					stop();
					servoaction(57);		//arms is brought down
					_delay_ms(150);
					forward();				//bot moves forward
					/*velocity(255,210);*/
					_delay_ms(300);
					backward();
					stop();
					magnet_on();			//magnet is turned on and pebble is picked up
					_delay_ms(130);
					servoaction(65);
					_delay_ms(200);
					servoaction(57);
					
					//CHECKING WHICH PEBBLE IS PICKED//
					if(s==0){
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 's';		}				//sending trigger for animating pebble pickup #1
					else if (s == 1){
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'g';						//sending trigger for animating pebble pickup #2
					}
					else if (s == 2){
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'h';						//sending trigger for animating pebble pickup #3
					}
					
					s++;
									
 					_delay_ms(200);
					 					
					backward();						//going back to continue line following
					_delay_ms(600);
					forward();
					stop();
					servoaction(10);				//arm is taken upwards to avoid collisions

					}
				else if(ins[j]=='d'){				//drop pebble

					backward();
					stop();
					servoaction(57);				//arm is brought down
					_delay_ms(150);

					forward();						//bot is moved forward


					_delay_ms(320);
					stop();
					magnet_off();					//pebble is dropped

					//checking which pebble is dropped
					if(s==1){
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'd';				}		//sending trigger for animating pebble drop #1
					else if(s == 2){
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'c';						//sending trigger for animating pebble drop #2
					}

					_delay_ms(100);

					backward();						//going back to continue line following

					_delay_ms(650);
					forward();
					stop();

					servoaction(10);				//arm is taken upwards to avoid collisions

				}
				else if(ins[j]=='f'){		//final pebble drop
					backward();
					stop();
					servoaction(57);		//arm is brought down
					_delay_ms(150);

					forward();				//bot is moved forward
					_delay_ms(300);
					stop();
					while(!(UCSR0A & TE));			//waiting to transmit
					UDR0 = 'f';						//sending trigger for animating final pebble drop 
					magnet_off();
					buzzer_on();					//buzzer is turned on for five seconds
					backward();						//bot is moved backwards so that the Aruco of the water pitcher is visible
					_delay_ms(480);
					forward();
					stop();
					_delay_ms(4500);
					buzzer_off();
					break;
				}
				
				else if(ins[j]=='a'){			//turn 180 degrees
					forward();
					_delay_ms(150);
					backward();
					stop();
					right();
					_delay_ms(700);
					velocity(255*0.6,240*0.6);
					/*flag=1;*/
					flaga = 1;

				}
				else if(ins[j]=='q'){			//turn right to align axis of the robot with the axis of the aruco marker

					forward();
					_delay_ms(140);

					left();
					velocity(255*0.6,245*0.6);
					_delay_ms(200);
					flag2 = 1;
					

				}
				else if(ins[j]=='w'){			//turn left to align axis of the robot with the axis of the aruco marker 					
					forward();
					_delay_ms(140);

					right();
					velocity(255*0.6,240*0.6);
					_delay_ms(200);
					flaga = 1; 					
				}
				else if(ins[j]=='u'){
					forward();
					_delay_ms(150);
					right();
					_delay_ms(217);



				}
				else if(ins[j]=='i'){
					forward();
					_delay_ms(150);
					left();
					_delay_ms(195);
					

				}
				j++;
				}						

    }
	if (Center_white_line<20  && Left_white_line<20 && Right_white_line<20){
		if(flag==2){
			backward();
		}
		else if(flag==0){
			PORTA = 0x04;
		}
		else if(flag==1){
			PORTA= 0x01;
		}

	}
}
