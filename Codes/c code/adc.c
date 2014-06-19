/********************************************************************************
 Written by: Aditya Sharma NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 13th January 2010
 
 In this experiment ADC captures the analog sensor values and displayes it on the LCD

 Concepts covered:  ADC, LCD interfacing

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

			  ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 LCD Display interpretation:
 ****************************************************************************
 *BATTERY VOLTAGE	IR PROX.SENSOR 2	IR PROX.SENSOR 3	IR.PROX.SENSOR 4*
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		FRONT SHARP DIS *
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 11059200
 	Optimization: -O0 (For more information read section: Selecting proper optimization options 
						below figure 4.22 in the hardware manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function


unsigned char ADC_Value;


//ADC pin configuration
void adc_pin_config (void){
  DDRF = 0x00; //set PORTF direction as input
  PORTF = 0x00; //set PORTF pins floating
  DDRK = 0x00; //set PORTK direction as input
  PORTK = 0x00; //set PORTK pins floating
}

//Function to Initialize PORTS
void display_port_init(){
  adc_pin_config();	
}

//Function to Initialize ADC
void adc_init(){
  ADCSRA = 0x00;
  ADCSRB = 0x00;		//MUX5 = 0
  ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
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
  while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}




// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
  float distance;
  unsigned int distanceInt;
  distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
  distanceInt = (int)distance;
  if(distanceInt>800)
    {
      distanceInt=800;
    }
  return distanceInt;
}

unsigned int Sharp_GP2D120_estimation(unsigned char adc_reading)
{
  float distance;
  unsigned int distanceInt;
  distance  = 10.00*((1.00/ ((0.001240875*(float) adc_reading) + 0.005))-0.42);
  distanceInt = (int)distance;
  if(distanceInt>300)
    {
      distanceInt=300;
    }
  return distanceInt;
}
