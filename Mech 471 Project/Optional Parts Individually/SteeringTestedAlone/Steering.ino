#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void steering_control(float yaw);
#define BIT(a) (1UL << (a))

void setup() 
{  
  // Serial startup sequence below ensures reliable / predictable startup /////
  char ch;
  Serial.begin(115200);
  
  // wait until Serial Monitor or save_serial program is started
  while( !Serial ) delay(1);
  while(1) {
    if( Serial.available() > 0 ) { // is there incoming data
      ch = Serial.read(); // read one character / byte
      if( ch == 's' ) break;
    }
    delay(10); // leave some time for printing message above, etc.
  }
  
  steering_control(90);
  
  //t0 = micros()*1.0e-6; // initial time (s)
  delay(1000);
  exit(0);
}

// takes in an angle in degrees, between 45-135;
void steering_control(float yaw)
{
  //timer parameters
  unsigned long int n;
  float t, tp, T, t_OF, k, time, time_next, pulse_duration;
  int pulse;
  int count_p, count;
  float angle_degrees;
  angle_degrees = yaw;
  
  //limit the angles to 45 to 135 degrees, 90 degrees is resting/straight position
  if(angle_degrees > 135){
    angle_degrees = 135;
  }
  else if (angle_degrees < 0){
    angle_degrees = 0;
  }

  pulse = ((0.5/90)*(angle_degrees-45)+1.25)*1000;
  // setup pin 11 to output
  DDRB |= BIT(2);

  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0B |= BIT(CS01) | BIT(CS00); // 64
  TCNT0 = 0; // reset
  TIMSK0 = 0;

  n = 0;
  T = 0.02; // 20 ms pulse period
  t_OF = 64/16.0e6*256; //duration for one overflow
  count_p = count = TCNT0;
  k = 64/16.0e6; // time per count
  t = k*count; // clock time, resets at overflow
  time = t + n*t_OF;
  time_next = time + T;
  
while(1){
    while(time < time_next){
      count = TCNT0;
      
      if(count < count_p) n++;

      time = t + n*t_OF;
      count_p = count;
    }

    cli();
    PORTB |= BIT(2);
    count = TCNT0;

    if(count < count_p) n++;
    time = t + n*t_OF;

    count_p = count;

    pulse_duration = time + pulse*1.0e-6;

    while(time < pulse_duration){
      count = TCNT0;
      
      if(count < count_p) n++;

      time = t + n*t_OF;
      count_p = count;
    }
    PORTB &= ~BIT(2);

    sei();
    time_next += T;
}
  
/*
  int n = 100;
  float y1 = -1, y2 = -1, y3 = -1;
  float w_back, w_fright, w_fleft;
  float u1, u2;
  int pw1, pw2;
  int w1, w2; 
  float L = 0.1; // meters
  float B = 0.05; // meters
  float Rsteer, Rback, Rrobot;
  float yaw_rad;
  float w_innerfront, w_outerfront, w_rear; // ang. vel. when rotating
  float tyre_radius = 0.025; // meters
  const float wmax = 35.0; // maximum back wheel speed (rad/s) 

  // voltage stuff
  const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000; 
  const float V_bat = 12.0; // battery voltage
  const float V_bat_inv = 1/V_bat;
  const float u_max = V_bat;
  const float u_min = -V_bat;
  
  // Variables for PID
  static float kp, kd, ki; // controller gains
  static float e, ed, ep = 0.0, ei = 0.0;; //error for controllers
  static float u; // modifies speed and sends a value to speed control
  static float t, dt, T; // time values for derivative with period
  static float tp = 0; // values conserved for multiple loops 
  const float tol = 1.0e-10;

  y1 = read_ADC_voltage(A1,n); //drive motor
  y2 = read_ADC_voltage(A3,n); //front right motor
  y3 = read_ADC_voltage(A5,n); //front left motor
 
 Serial.print(y1);
 Serial.print(", ");
 Serial.print(y2);
 Serial.print(", ");
 Serial.print(y3);
 Serial.print(", ");
 //https://www.ri.cmu.edu/pub_files/pub3/shamah_benjamin_2001_1/shamah_benjamin_2001_1.pdf
 
  // back wheel angular velocity (rad/s)
  w_back = (y1 - 2.5) * 0.4 * wmax;
  // front right wheel angular velocity (rad/s)
  w_fright = (y2 - 2.5) * 0.4 * wmax;
  // front left wheel angular velocity (rad/s)
  w_fleft = (y3 - 2.5) * 0.4 * wmax;

  //Convert degrees to rad, and limit to 65 degree turn
  if(yaw > 65) yaw = 65.0;
  else if(yaw < 0) yaw = 0.0;
  yaw_rad = yaw*(PI/180);

  //steering equations
  Rsteer = L/sin(yaw);
  Rback = sqrt(Rsteer*Rsteer-L*L);
  Rrobot = sqrt(Rback*Rback+(L/2)*(L/2));
  
  w_innerfront = (v/(2*PI*tyre_radius)*((Rsteer-B/2)/Rrobot));
  w_outerfront = (v/(2*PI*tyre_radius)*((Rsteer+B/2)/Rrobot));
  w_rear = (v/(2*PI*tyre_radius)*((Rback)/Rrobot));

  if(left_or_right == 0){
    w_fright = w_outerfront;
    w_fleft = w_innerfront;
    w_back = w_rear;
  }
  else if(left_or_right == 1){
    w_fright = w_innerfront;
    w_fleft = w_outerfront;
    w_back = w_rear;
  }
  
  // PID gains
  kp = 10.0; 
  kd = 0.0; 
  ki = 0.0; 
  
  //t = micros()*1e6 - t0;
  dt = t - tp;
  tp = t;
  
  e = r - y;

  T = 0.014; //keep a constant period for now
  ei += e*T; // integral of the error
  ed = (e - ep)/T; // derivative of error
  ep = e;
  
  //PID
  u = kp*e + ki*ei + kd*ed; 
  if( u > u_max ) u = u_max;
  if( u < u_min ) u = u_min; */
}

void loop() {
  // put your main code here, to run repeatedly:

}
