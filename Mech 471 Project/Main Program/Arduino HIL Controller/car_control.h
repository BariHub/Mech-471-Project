#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <Arduino.h>
#include "microcontroller_functions.h"

// important variables
//volatile float voltage_G;
//const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
//const float wmax = 35.0; // maximum car motor speed (rad/s)
//const float V_bat = 12.0; // lipo battery voltage
//const float V_bat_inv = 1/V_bat;	
void collision_control();
float speed_control(float r);
float traction_control(float r);
//call's ADC function?
float get_speed_in_volts_tachometer();

//Function: speed controller
//input: calls get_speed_in_volts_tachometer();
//output: voltage
float speed_control();
float calculate_reference_speed();
//Function: traction control version1 -> will need editing
//input: forward velocity and back wheels velocity in voltage
//output: voltage?
float traction_braking_control(float forward_velocity_volt, float back_velocity_volt);

//Function: translate voltage to time
//input: voltage in Volts
//output: time in microseconds
float volt_to_time(float u);
float PID_selector(float reference_speed);
int calc_steering();
#endif
