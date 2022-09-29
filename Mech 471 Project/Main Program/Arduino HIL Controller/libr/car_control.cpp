#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//variables
volatile float voltage_G;
const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
const float wmax = 35.0; // maximum car motor speed (rad/s)
const float V_bat = 12.0; // lipo battery voltage
const float V_bat_inv = 1/V_bat;  



//call ADC function?
float get_speed_in_volts_tachometer(){
float v  =  read_ADC_voltage_register(A0, 200);
return v;
}

//Function: speed controller
//input: calls get_speed_in_volts_tachometer();
//output: voltage
float speed_control(){
	int n = 200; // number of ADC samples in averaging filter
	float drive_motor = -1, right_front_motor = -1, left_front_motor = -1;
	float w;
	float y,u,kp,e,ed,u_max,u_min;
	float u1, u2;
	int pw1, pw2; 	//pw1 = output i  pwm
	int w1, w2;	//w1 motor volts to pwm
	float t1, t;
  static float r; //desired value angular velocity (whatever w is ) rad/s
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 35.0; // maximum car motor speed (rad/s)
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;	
	const float ADC_to_V = 1.0/1023.0*5;
	float ki,kd,T,rd;
	static float ei = 0.0, ep = 0.0;
	static float dt, tp = -1.0e6; 


	// read car outputs ///////////////
	t1 = micros();
	//this needs to change for coherency
	drive_motor =get_speed_in_volts_tachometer() ;
 // right_front_motor = read_ADC_voltage_register(A0,n);
	//left_front_motor = read_ADC_voltage_register(A5,n);
  	// current time (us)
  	t = micros();	

        // length of ADC interval (us)
        dt = t - t1; // for adjusting / optimizing the sensor filter

	// step 2) calculate the control input ///////  
	kp = 2.0; 
	kd = 0.0; 
	ki = 0; 

	// use the following scale for the output drive_motor
	// w = 0 	-> drive_motor = 2.5 V
	// w = wmax 	-> drive_motor = 5.0 V
	// drive_motor = 2.5 + w * wmax_inv * 2.5 ->
	w = (drive_motor - 2.5) * 0.4 * wmax; //the equationvoltage to w
  Serial.println(w);
	// calculate inputs
	//find the desired voltage input
	// step input for u1 at t = 5s and down to zero at t = 10s
	if(t > 100) {
		r = 0.0;
	} else if (t > 5) {
		r = 5.0;
	} else {
		r = 0.0;
	}

	e = r - drive_motor; // controller error
	T = 0.014; //keep a constant period for now
	ei += e*T; // integral of the error
	//T = t*1.0e-6 - tp; // controller period in seconds
	ed = (e - ep)/T; // derivative of error
	ep = e;
	//tp = t*1.0e-6; // time in seconds
  
  	u = kp*e + ki*ei + kd*ed; // PID controller
  
 
  	// use max u_max less than V_bat for safer initial testing
  	u_max = 11;  // u_max <= V_bat
  	u_min = -11; // u_min >= -V_bat
  
  	// software saturation of inputs
  	if( u > u_max ) u = u_max;
  	if( u < u_min ) u = u_min;
	
	return u; //u1 equivalence
	//if we need to return u1 and u2 in the future
	// we're pass them in by reference
	

}

//Function: traction control version1 -> will need editing
//input: forward velocity and back wheels velocity in voltage
//output: voltage?
float traction_braking_control(float forward_velocity_volt, float back_velocity_volt){

  static float delta_traction; // for tacho feedback averaging
  static float y, rmax, rmin, r; // slip ratio, real and desired, + erorr
  static float kp, kd, ki; // controller gains
  static float e, ed; //error for controllers
  static float u; // modifies speed and sends a value to speed control
  static unsigned long int t, dt; // time values for derivative
  static unsigned long int tp = micros(); // values conserved for multiple loops
  static float ep = 0.0, ei = 0.0;
  static float T; // period: time it takes to do one loop

  kp = 5.0; 
  kd = 0.0; 
  ki = 100.0; 

  rmin = 0.1; rmax = 0.5; // range of values for acceptable slip

  r = (rmin + rmax) / 2; // average slip ratio to aim for

  if (back_velocity_volt >= forward_velocity_volt) 
    y = (back_velocity_volt - forward_velocity_volt) / back_velocity_volt; // traction
  else{ 
    y = (back_velocity_volt - forward_velocity_volt) / forward_velocity_volt; 
    r = -r; // braking
  }
  if ((abs(y) < rmin) | (abs(y) > rmax)) 
    delta_traction = 1; // traction control only active if beyond range
  else 
    delta_traction = 0;

  t = micros(); dt = t - tp;

  e = r - y;
  if (fabs(e) < 0.01) {
    e = 0;
  }
  ei += e * T;
  ed = (e - ep) / dt;
  u = (kp * e + kd * ed) * delta_traction + ki * ei;
  tp = t;
  ep = e;
  return u; //voltage

}

//Function: translate voltage to time
//input: voltage in Volts
//output: time in microseconds
float volt_to_time(float u){

	float w = u * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	return w;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////INTERUPTS///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
