/********************************************************************************
 Written by: Sachitanand Malewar NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 13th January 2010
 
 This experiment demonstrates robot velocity control using PWM.

 Concepts covered:  Use of timer to generate PWM for velocity control

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 
  
 Connection Details:  	L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 


 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 11059200
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

 2. Sudden change in the motor direction produces current surge up to 1.6Amps.
 	Battery can give current up to 3Amps. Auxiliary supply can give current up to 1.5Amp at peak load
	When robot is on Battery power we can do sudden change motion direction as battery can supply current up to 3Amps
    Motors are stopped for 0.5 seconds between consecutive motion commands to keep current surge below 1.5Amp.
	Stop motors for at least 0.3 seconds before reversing its direction if it is running at full speed.
*********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function

void motion_pin_config (void){
  DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
  PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
  DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
  PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}
//Function to configure ports to enable robot's motion

//Function to initialize ports
void motion_port_init(){
  motion_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:64
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:674.988Hz
void timer5_init()
{
  TCCR5B = 0x00;	//Stop
  TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;	//Output compare register high value for Left Motor
  OCR5AL = 0xFF;	//Output compare register low value for Left Motor
  OCR5BH = 0x00;	//Output compare register high value for Right Motor
  OCR5BL = 0xFF;	//Output compare register low value for Right Motor
  OCR5CH = 0x00;	//Output compare register high value for Motor C1
  OCR5CL = 0xFF;	//Output compare register low value for Motor C1
  TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
			  For Overriding normal port functionality to OCRnA outputs.
			  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
  TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
  unsigned char PortARestore = 0;

  Direction &= 0x0F; 			// removing upper nibbel as it is not needed
  PortARestore = PORTA; 			// reading the PORTA's original status
  PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
  PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
  PORTA = PortARestore; 			// setting the command to the port
}


void forward (void){ //both wheels forward
  motion_set(0x06);
}

void back (void){ //both wheels backward
  motion_set(0x09);
}

void left (void){ //Left wheel backward, Right wheel forward
  motion_set(0x05);
}

void right (void){ //Left wheel forward, Right wheel backward
  motion_set(0x0A);
}

void soft_left (void){ //Left wheel stationary, Right wheel forward
  motion_set(0x04);
}

void soft_right (void){ //Left wheel forward, Right wheel is stationary
  motion_set(0x02);
}

void soft_left_2 (void){ //Left wheel backward, right wheel stationary
  motion_set(0x01);
}

void soft_right_2 (void){ //Left wheel stationary, Right wheel backward
  motion_set(0x08);
}

void stop (void){
  motion_set(0x00);
}











