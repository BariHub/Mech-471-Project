 float measured_speed;
#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


void setup() {
  double t;
  float pwm =0;
  Serial.begin(115200);
  delay(3000);
float  ref_speed = 100.0;//hard coded just in case potentiometer is not available
  pin_setup(9, OUTPUT);
  while(1){ 
    int steering = calc_steering();//optional section/can comment out and hard code the angle 
    steering_control(steering);//optional section
   ref_speed= calculate_reference_speed();//can comment out and hard code the speed
   Serial.println(ref_speed);
    PID_selector(ref_speed);
 pwm = volt_to_time(PID_selector(ref_speed)); 
   // collision_control();//Optional section only uncomment if sensor is plugged in
 //Serial.println(micros());
// Serial.printl
//sei();
  set_pwm(9,pwm , 0.02);
  
 // Serial.println(micros());
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
