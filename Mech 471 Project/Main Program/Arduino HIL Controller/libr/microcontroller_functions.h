#ifndef MICROCONTROLLER_FUNCTIONS_H
#define MICROCONTROLLER_FUNCTIONS_H
#include <Arduino.h>
#include "car_control.h"

#define BIT(a) (1UL << (a))

//Function: create a pwm signal 
//Input: pin 3, 11, 9, 10; the duty cycle in MICROSECONDS, and the period in SECONDS
//Output: N/A
void set_pwm(int pin, float duty, float period);


//Function: initialize pin
//Input: pin 0 -> 13, mode: input, output, input_pullup
//Output: N/A
void pin_setup(int pin, int mode);


//Function: Analog to Digital Conversion
//Input: pin A0 -> A5
//Output: average reading in voltage
float read_ADC_voltage_register(int channel, int n);


#endif
