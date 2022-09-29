#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//variables
volatile float voltage_G;
const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
const float wmax = 810.0; // maximum car motor speed (rad/s)
const float V_bat = 12.0; // lipo battery voltage
const float V_bat_inv = 1/V_bat; 
const float ratio = 0.002; 
static float measured_speed;
const int NMAX=64;
static char buffer[NMAX];


//call ADC function?
float get_speed_in_volts_tachometer(){
float v  =  read_ADC_voltage_register(A3, 200);//measuring front wheels since it's directly proportional to actual vehicle speed which is what we actually want to control
return v;
}

//Function: speed controller
//input: calls get_speed_in_volts_tachometer();
//output: voltage
float speed_control(float r){
	int n = 200; // number of ADC samples in averaging filter
	float drive_motor = -1, right_front_motor = -1, left_front_motor = -1;
	float w;
	float y,u,kp,e,ed,u_max,u_min;
	float u1, u2;
	int pw1, pw2; 	//pw1 = output i  pwm
	int w1, w2;	//w1 motor volts to pwm
	float t1, t;
  static float t0; //desired value angular velocity (whatever w is ) rad/s
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum car motor speed (rad/s)
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
//  Serial.print("Drive motor : ");
  //Serial.println(drive_motor);
   // right_front_motor = read_ADC_voltage_register(A0,n);
	//left_front_motor = read_ADC_voltage_register(A5,n);
  	// current time (us)
  	t = micros()-t0;	
    Serial.print((t/1000000)-3);
    Serial.print(",");

        // length of ADC interval (us)
        dt = t - t1; // for adjusting / optimizing the sensor filter

	// step 2) calculate the control input ///////  
	kp = 0.6;
	kd = 0.0; 
	ki =  0.6; 
 
	// use the following scale for the output drive_motor
	// w = 0 	-> drive_motor = 2.5 V
	// w = wmax 	-> drive_motor = 5.0 V
	// drive_motor = 2.5 + w * wmax_inv * 2.5 ->
	w = (drive_motor - 2.5) * 0.4 * wmax; //the equationvoltage to w
   measured_speed = w;
  //Serial.print("w= ");
double ei_max;
 if (ki > 0.0) {
   ei_max = 0.14 * 12 / ki; // want ki*ei < 0.14*v_batt
 }
 else {
   ei_max = 0.0;
 }
  double z;
 if ((ei > ei_max) && (e > 0)) {
   z = 0; 
 }
else if ((ei < -ei_max) && (e < 0)) { 
  z = 0; 
 }
 // else { 
   // z = e; 
  //}
  Serial.print(w);
  Serial.print(",");
	// calculate inputs
	//find the desired voltage input
	// step input for u1 at t = 5s and down to zero at t = 10s
//	if(t > 100000000) {
	//	r = 200.0;
    
//	} else if (t > 300000) {
	//	r = 200.0;
    
	//} else {
		//r = 0.0;
	//}
//Serial.print(r);
//Serial.print(",");
	e = r - w; // controller error
	T = 0.0175; //keep a constant period for now
	ei += e*T; // integral of the error
	//T = t*1.0e-6 - tp; // controller period in seconds
	ed = (e - ep)/T; // derivative of error
	ep = e;
	//tp = t*1.0e-6; // time in seconds

 Serial.print(e);
 Serial.print(",");
 	u = kp*e+kd*ed+ki*ei ; // PID controller
  //u = 6;
  	// use max u_max less than V_bat for safer initial testing
  	u_max = 11.5;  // u_max <= V_bat
  	u_min = -11.5; // u_min >= -V_bat
  
  	// software saturation of inputs
  	if( u > u_max ) u = u_max;
  	if( u < u_min ) u = u_min;
	//Serial.print("u: ");
  Serial.print(u);
  Serial.println(",");
  
	return u; //u1 equivalence
	//if we need to return u1 and u2 in the future
	// we're pass them in by reference
	

}
float traction_control(float r){
  //set all the variables up
  int n = 200;
  float y1 = -1, y2 = -1, y3 = -1;
  float wb, wf, w_front;
  float u_front, u2; //voltage output
  float u_max;
  float u_min;
  int pw_front, pw2; //microseconds output
  const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
  const float wmax = 810.0; // maximum back wheel speed (rad/s) 
  const float V_bat = 12.0; // lipo battery voltage
  const float V_bat_inv = 1/V_bat;  
  static float slip_measured;
  static float slip_ratio;
  static float kp, kd, ki; // controller gains
  static float e, ed, ep = 0.0, ei = 0.0;; //error for controllers
  static float u; // modifies speed and sends a value to speed control
  static float t, dt; // time values for derivative
  static float tp = 0; // values conserved for multiple loops 
  static float T; // period: time it takes to do one loop
  float t0;

  //Get Inputs
  //read the speeds in voltage (system input)
  y1 = read_ADC_voltage_register(A1,200); //drive motor
  y2 = read_ADC_voltage_register(A3,200); //front right motor
  //  y3 = read_ADC_voltage_regsiter(A5,n); //front left motor

  Serial.println("here");
  // back wheel angular velocity (rad/s)
  wb = (y1 - 2.5) * 0.4 * wmax;

  // front wheel angular velocity (rad/s)
  wf = (y2 - 2.5) * 0.4 * wmax;
  measured_speed = wf;

  //calculate the slip based on the front left and drive wheels
  slip_measured = (wb - wf)/fabs(wb+0.0001); //traction

  // read clock time (s)
  t = micros()*1.0e-6 - t0;

  //PID CONTROLLER SECTION/////

  //Set the input and output of the PID
  //r = 0.2 for acceleration r= -0.2 braking
  float y = slip_measured;

  //pid gains
  kp = 300; 
  ki = 80; 
  kd = 0; 
  dt = t - tp;
  tp = t;

  //calculate error, and if its within +/- 0.01 set to 0
  e = r - y; //refernce slip - calculated input slip
  if (fabs(e) < 0.01) {
    e = 0;
  }
  //calculate the pid errors 
  T = 0.014; //keep a constant period for now
  
  //based on the dynamixel anti wind up logic 
  double ei_max;
  if (ki > 0.0) ei_max = 0.14 * 12 / ki; // want ki*ei < 0.14*v_batt
  else ei_max = 0.0;
  double z;
  if ((ei > ei_max) && (e > 0)) z = 0; 
  else if ((ei < -ei_max) && (e < 0)) z = 0; 
  else z = e; 

  ei += z * T;
  ed = (e - ep)/T; // derivative of error
  ep = e;
if(e<10.00){
  
  }
  //slip output from PID
  u = kp*e + ki*ei + kd*ed; 
  //u = e;
    u_max = 11;  // u_max <= V_bat
    u_min = -11.0; // u_min >= -V_bat
  
    // software saturation of inputs
    if( u > u_max ) u = u_max;
    if( u < u_min ) u = u_min;
  u_front = u;

  // convert motor voltage to pulse width
  //w_front = u_front * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
  
  // make sure its within bounds
  if(w_front > PW_MAX) w_front = PW_MAX;
  if(w_front < PW_MIN) w_front = PW_MIN;  
  
  pw_front = w_front;
  // set pw2 for testing purposes
 // pw2 = 1500;
  
  // set car actuators
  //pwm(7,pw_front); //front wheel
 // pwm(8,pw2); //back
  
  // print out for testing purposes
  
 Serial.print(t);
  Serial.print(",");
  
  Serial.print(r);
  Serial.print(",");
  
  Serial.print(y);
  Serial.print(",");

  Serial.print(e);
  Serial.print(",");  
//  
  Serial.print(u_front);
  Serial.print("\n");
  
//  delay(30);
return u_front;
}
//

//Function: translate voltage to time
//input: voltage in Volts
//output: time in microseconds
float volt_to_time(float u){

	float w = 1*(u+11)/(23) ; 
 // w = w*ratio;
  //Serial.print("time: ");
  //Serial.print(w);
	return w;
}

float PID_selector(float reference_speed){
//get speed reading of front wheel in rad/s 
float pwm;
static double t;
t= micros()/1000000;
//is the speed measured 50% higher than the desired?
if(reference_speed*1.5 < measured_speed&&t<0.5)pwm= traction_control(-0.4); //we need to start braking, r=-0.2

//is the speed measured 50% lower than the desired?
else if(reference_speed *0.5 > measured_speed && t<0.4) pwm=traction_control(0.4); //we need to accelerate, r=0.4, only used during launch 

//if within the 3% tolerance, control the speed

else
pwm =speed_control(reference_speed);

//Serial.print(measured_speed);
//Serial.print(",");
//Serial.print(reference_speed);
//Serial.print("\n");
return pwm;
//output some speeds and tractions

}

int calc_steering(){
  int n;
  int len;
  static int ref_steer =90;
  int *theta;
  if (Serial.available()>0){                                        /////////////////reading data from keyboard////////////////////////
   len=3;
   
   n = Serial.readBytes(buffer,len);
}
 // theta = (int*)(buffer+2);
  //Serial.print("theta= ");
 // Serial.println(buffer[2]);
if (buffer[2]=='8'){                                                    //////////////////////////setting motor speed for increase if up button is pressed/////////////////
  ref_steer = ref_steer+2; 
}

else if (buffer[2]=='t'){                                                   //////////////////////////setting motor speed for increase if up button is pressed/////////////////
 ref_steer = ref_steer-2;
}
else 
ref_steer =ref_steer;
Serial.print("angle: ");
Serial.println(ref_steer);
  return ref_steer;
  
  }

float calculate_reference_speed(){
  float reference_speed;
  float voltage =read_ADC_voltage_register(A5, 200);
  reference_speed = voltage/5.0 *810;
  Serial.print("Reference Speed ");
  Serial.println(reference_speed);
return reference_speed;
 
  
  }


 void collision_control() 
{   
  float pwm;
float voltage = read_ADC_voltage_register(A0,200);
pwm = map(voltage,622,62,0,5);
if(pwm>600)
set_pwm(9,0 , 0.02);//when distance between the sensor and an object in front of it is less than around 1 meter stop moving forward
}



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////INTERUPTS///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
