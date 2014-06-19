/**@mainpage package lcd_interface_is_my_project_name
 @author Group 7: Garvit Juniwal 08005008
				   Ravi Bhoraskar 08005002
				   Kunal Shah 08005005
				   Namit Katariya 08005007
 
 AVR Studio Version 4.17, Build 666

 Date: 7th April 2010
 
*********************************************************************************/


/********************************************************************************

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"
#include "buzzer.c"
#include "adc.c"
#include "motion.c"
#include "servo.c"
#include "xbee.c"

#define FCPU 11059200ul //defined here to make sure that program works properly




//Function to configure LCD port
void lcd_port_config (void){
  DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output 11110111
  PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7 00001000
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
  ADC_Value = ADC_Conversion(channel);
  lcd_print(row, coloumn, ADC_Value, 3);
}




/**** serial communication starts here ***/
//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled


unsigned char data = 0x00;
int wait_for_interrupt = 1;
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
  wait_for_interrupt = 0;
  data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

  UDR0 = data; 				//echo data back to PC
	
}

#define MOVE_FORWARD_FOR 50 //in ms, we excute the last commmand from server this many number of times 
#define EXTRA_DELAY_FOR_HARD 50//in ms, in case of hard left or right, we give slight extra delay because a hard turn is slower then moving forward

void execute_last_command(void)
{
  switch(data){
  case 0x00://forward
    velocity(255,255);
    forward();
    break;
  case 0x01://soft left
    velocity(64,255);
    forward();
    break;
  case 0x02://hard left
    velocity(255,255);
    left();
	_delay_ms(EXTRA_DELAY_FOR_HARD);
    break;
  case 0x03://soft right
    velocity(255,64);
    forward();
    break;
  case 0x04://hard right
    velocity(255,255);
    right();
	_delay_ms(EXTRA_DELAY_FOR_HARD);
    break;
  case 0x06://hard left but for little time in cases where we need fine tuning
 	velocity(255,255);
 	left();
 	break;
  case 0x07://hard right but for little time in cases where we need fine tuning
	velocity(255,255);
 	right();
 	break;
  default:
    stop();						
  }
  _delay_ms(MOVE_FORWARD_FOR);
  stop();
}



/** serial communication ends here ***/

#define READ_INTERVAL .01 //in sec, the amount of time to wait between consecutive readings while averaging
#define NUM_SAMPLES_FOR_AVERAGE 10
#define NUM_SAMPLES_FOR_INITIAL_AVERAGE 100
#define THRESHOLD_COUNT 2//the number of times for which we should get a low road depth estimate to conclude the presence of a hole
#define THRESHOLD 10 //in mm, the minimum depth of  hole. if the depth is greater than this depth, then only hole is detected
#define FILLINGTHRESHOLD 5 // in mm, we want to slightly overfill the hole by this height, as would happen in a real scenario
#define FILLINGTHRESHOLD_COUNT 2// the number of times a low reading of depth should occur to conclude that a hole is filled

#define EXE_LAST_COMMAND_NUM 5 // execute the last command from the server this number of times
#define OPEN_FOR 350 //in ms, the time for which to open the shutter in one shot. we expect to drop 1-2 balls each time we open the shutter

unsigned char open = 90;
unsigned char close = 60;



void open_hopper_shutter(){
	servo_1(open);
	_delay_ms(OPEN_FOR);//open hopper's shutter for 350 ms. This value is based on testing.
	servo_1(close);
}


int road_depth_estimate = 160; //height of sensor from ground, but will be recomputed to avoid claibration problem.
void fill_hole(){
  execute_last_command();//just to bring hopper in line with the hole center.
  unsigned char sharp;
  unsigned int value=0, avg=0;
  
  int count=0;
  while(1){
	open_hopper_shutter();
	value=0;
    for(int i=0; i<NUM_SAMPLES_FOR_AVERAGE; i++){
      sharp = ADC_Conversion(12);			//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
      value += Sharp_GP2D120_estimation(sharp);		//Stores Distance calsulated in a variable "value".
      //lcd_print(2,10,value,3); 			//Prints Value Of Distanc in MM measured by Sharp Sensor.
      _delay_ms(READ_INTERVAL*1000);
    }
    avg = (int)((1.0*value)/NUM_SAMPLES_FOR_AVERAGE);
    if(avg < road_depth_estimate - FILLINGTHRESHOLD){
      count++;
      if(count>FILLINGTHRESHOLD_COUNT){
		buzzer_off();
		servo_1(close);
		return;	
      }
    }
	
  }
}   

void take_initial_estimate(void){
  unsigned char sharp;
  unsigned int value=0;
  	
  
  for(int i=0; i<NUM_SAMPLES_FOR_INITIAL_AVERAGE; i++){
    sharp = ADC_Conversion(12);			//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
    value += Sharp_GP2D120_estimation(sharp);		//Stores Distance calsulated in a variable "value".
    //lcd_print(2,10,value,3); 			//Prints Value Of Distanc in MM measured by Sharp Sensor.
    _delay_ms(READ_INTERVAL*1000);
  }
  road_depth_estimate = (int)((1.0*value)/NUM_SAMPLES_FOR_INITIAL_AVERAGE);

}

/******************************************************************************************************************/
/* POTHOLE DETECTION ALGORITHM: 		    							          */
/* At fixed amounts of time, the bot stops and takes the "average" reading of the depth of the road. 		  */
/* The averaging procedure is as follows: Take the sharp sensor reading at fixed intervals of time and add them.  */
/* (Note: The bot is at the same place while these multiple readings are being taken) Divide it by the number 	  */
/* of samples taken to get the average reading at that time. 							  */
/*   The algorithm has at its disposal the estimate of the current depth of the road in the 			  */
/* variable "road_depth_estimate" (given by the function take_initial_estimate()). If the current average 	  */
/* reading is greater than the current estimate plus a certain THRESHOLD (set by us), then we increment a 	  */
/* counter. Here this counter is the "consecutive_falls_in_avg". The name of the counter provides an insight 	  */
/* into the pothole_detection algorithm. We infer that a hole has been detected when this counter reaches a 	  */
/* THRESHOLD_COUNT i.e when the average depth consecutively decreases for a THRESHOLD_COUNT number of times. 	  */
/* If the average is less than depth_estimate+THRESHOLD, we reset the counter.					  */
/******************************************************************************************************************/
unsigned int consecutive_falls_in_avg=0;
void sample_readings(){  
  unsigned char sharp_sensor_reading; //raw reading taken from the sharp sensor
  unsigned int sum_distance=0; //variable to store the cumulative distance over a number of samplings. sum_distance / no. of samples taken will yield the average reading
  unsigned int avg_distance=0; // average distance over a fixed number of readings
  stop();
  for(int i=0; i<NUM_SAMPLES_FOR_AVERAGE; i++){
    sharp_sensor_reading = ADC_Conversion(12); //Stores the Analog value of front sharp sensor connected to ADC channel 11 into variable "sharp_sensor_reading"
    sum_distance += Sharp_GP2D120_estimation(sharp_sensor_reading); //distance reading is cumulatively added for the purpose of averaging. This will be divided by the number of readings taken to get the average distance. 
    //lcd_print(2,10,sum_distance,3); //Prints Value Of Distance in MM measured by Sharp Sensor.
    _delay_ms(READ_INTERVAL*1000); //sharp sensor reading taken at intervals of (READ_INTERVAL*1000) seconds. 
  }
  avg_distance = (int)((1.0*sum_distance)/NUM_SAMPLES_FOR_AVERAGE); // average distance
  lcd_print(1,10,avg_distance,3); //print the average distance calculated on the lcd
  lcd_print(1,14,road_depth_estimate,3); //print hte current estimate of the road depth on the lcd
  if(avg_distance > road_depth_estimate + THRESHOLD) //if the current average distance is greater than the road depth + fixed threshold, the road is deeper here than usual. Hence increment "consecutive_falls_in_avg" counter
    consecutive_falls_in_avg++;
  else if(avg_distance <= road_depth_estimate + THRESHOLD) //if the depth of the road decreases i.e the road's height has increased, reset the counter to zero
    consecutive_falls_in_avg=0;
  if(consecutive_falls_in_avg == THRESHOLD_COUNT){ //pothole detected: Ring buzzer    
    buzzer_on();
    fill_hole(); //iterative filling of the hole using the hopper mechanism
  }
}

void init_devices (void)
{
  cli(); //Clears the global interrupts
  display_port_init();
  lcd_port_config();
  adc_init();
  buzzer_port_init();
  motion_port_init();
  timer1_init();
  servo_port_init();
  timer5_init();
  uart0_init();
  sei(); //Enables the global interrupts
}


//Main Function
int main(void)
{
  init_devices();
	
  lcd_set_4bit();
  lcd_init();
  
  servo_1(close);	
  take_initial_estimate();
  
  while(1)
    {
      sei();
      while(wait_for_interrupt);
      cli();
      wait_for_interrupt = 1;
      int count_moves = 0;
      for(;count_moves<EXE_LAST_COMMAND_NUM;count_moves++){
		sample_readings();
		execute_last_command();
      }
    }
}
