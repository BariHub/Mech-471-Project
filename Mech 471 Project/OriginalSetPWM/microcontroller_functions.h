#ifndef MICROCONTROLLER_FUNCTIONS_H
#define MICROCONTROLLER_FUNCTIONS_H
#include <Arduino.h>
#include "car_control.h"

#define BIT(a) (1UL << (a))

//Function: create a pwm signal 
//Input: any digital pin; the duty cycle in microseconds
//Output: N/A
void pwm(int pin, int duty);

//Function: set the period of the PWM in microseconds
//Input: period in microseconds
//Output: N/A
void set_period(double period);

//Function: setup timer1 for our pwm function 
//Input: N/A
//Output: N/A
void pwm_setup_timer1();

//Function: get the time since the start in microseconds
//Input: N/A
//Output: time in microseconds since program began
double get_time_us();

//Function: setup timer 1
//Input: N/A
//Output: N/A
void get_time_us_setup();

//Function: initialize pin
//Input: pin 0 -> 13, mode: input, output, input_pullup
//Output: N/A
void pin_setup(int pin, int mode);


//Function: Analog to Digital Conversion
//Input: pin A0 -> A5
//Output: average reading in voltage
float read_ADC_voltage_register(int channel,int n);


#endif
