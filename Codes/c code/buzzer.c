#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function

//Function to initialize Buzzer 
void buzzer_pin_config (void){
  DDRC = DDRC | 0x08;		//Setting PORTC 3 as output 00001000
  PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer 11110111
}

void buzzer_port_init (void){
  buzzer_pin_config();
}

void buzzer_on (void){
  unsigned char port_restore = 0;
  port_restore = PINC;
  port_restore = port_restore | 0x08;
  PORTC = port_restore;
}

void buzzer_off (void){
  unsigned char port_restore = 0;
  port_restore = PINC;
  port_restore = port_restore & 0xF7;
  PORTC = port_restore;
}
