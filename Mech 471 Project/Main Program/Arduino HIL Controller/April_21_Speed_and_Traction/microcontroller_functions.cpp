#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//set variables
volatile double t2_offset = 0.0;
float z_3 = 50;
volatile float z_9 = 50;
float t0;
long volatile int sum_ADC=0;
volatile int count=0;
volatile unsigned int Ninterrupts;
//Function: create a pwm signal 
//Input: pin 3, 11, 9, 10; the duty cycle, and the period
//Output: N/A
void set_pwm(int pin, float duty, float period){
  //we don't want to mess around with timer 0, so pins 5 and 6 will not be used
    static unsigned int match_register;
          
  long cycles=0; 
  if(period > 0) {
    static int firsttime=1;
    cycles = (F_CPU * period) / 2000000;
    //to do: check which pin is A and which pin is B 
    //in order to be able to use all pwm pins at the same time
    if(pin == 9 ){
        //z_9 = duty/4;
        cli();
        if(firsttime==1){
        TCCR1A = 0;
        TCCR1B = 0;
        firsttime =2;
        TCCR1B |= BIT(WGM12);
        TCCR1B |= BIT(CS11) | BIT(CS10); //prescaler 64
        match_register = period *(16*pow(10, 6)) / 64;
        }
        //calculate period 
        //period in seconds = match_register * 64 /16e6
       // unsigned int match_register = period *(16*pow(10, 6)) / 64;
        OCR1A = match_register; //match_register;
        z_9 = duty*OCR1A;
        //Serial.print(OCR1A);
        TCNT1 = 0;
        TIFR1 |= BIT(OCF1A);
        TIMSK1 = BIT(OCIE1A);  
        sei();
        TCNT1 = 0;             
    }
    else if (pin == 10){
      
    }
    else if(pin == 3 ){
        z_3 = duty;
        TCCR2A = 0;
        TCCR2B = 0;
        TCCR2A |= BIT(WGM12);
        TCCR2B |= BIT(CS11) | BIT(CS10); //prescaler 64
        //calculate period 
        //period in seconds = match_register * 64 /16e6
       match_register = period *(16*pow(10, 6)) / 64;
        //Serial.print(match_register);
        OCR2A = match_register;
        z_3 = duty*OCR2A;
        TCNT2 = 0;
        TIFR2 |= BIT(OCF1A);
        TIMSK2 = BIT(OCIE1A);  
        sei();
        TCNT2 = 0;                                                 
    }
  }

}




//Function: initialize pin
//Input: pin 0 -> 13, mode: input, output, input_pullup
//Output: N/A
void pin_setup(int pin, int mode)
{
  int i;
  //specify the bit that will be turned on or off with a switch
  switch(pin){
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    i = pin;
    break; 
    case 8:
    case 9:
    case 11:
    case 12:
    case 13:
    i = pin - 8;
    break;
  };
  
  
  //PORT B
  if(pin>=8 && pin<=13){
      if(mode == INPUT){
        int oldSREG = SREG;
        cli();
        DDRB &= ~BIT(i);
        SREG = oldSREG;}
      else if (mode == INPUT_PULLUP){
        int oldSREG = SREG;
        cli();
        DDRB &= ~BIT(i);
        SREG = oldSREG;}
      else if(mode == OUTPUT){
        int oldSREG = SREG;
        cli();
        DDRB |= BIT(i);
        SREG = oldSREG;
        }
  }
  
  //PORT D
  if(pin>=0 && pin<=7){
      if(mode == INPUT){
        int oldSREG = SREG;
        cli();
        DDRD &= ~BIT(i);
        SREG = oldSREG;}
      else if (mode == INPUT_PULLUP){
        int oldSREG = SREG;
        cli();
        DDRD &= ~BIT(i);
        SREG = oldSREG;}
      else if(mode == OUTPUT){
        int oldSREG = SREG;
        cli();
        DDRD |= BIT(i);
        SREG = oldSREG;}
  }
  
  
}


//Function: Analog to Digital Conversion
//Input: pin A0 -> A5
//Output: average reading in voltage




  const float ADC_to_V = 1.0/1023.0*5;
float read_ADC_voltage_register(int channel,int n){
// Serial.println(micros());
  cli();
  int i; 
  float input, voltage;

  Ninterrupts = 150;
  sum_ADC =0;
  count =0;
  ADMUX =0;
  switch (channel){
    case A0:
    break;
  
    case A1:
    ADMUX |= BIT(MUX0);
    break;
    
    case A2:
    ADMUX |= BIT(MUX1);
    break;

    case A3:
    ADMUX |= BIT(MUX1)| BIT(MUX0);
    break;

    case A4:
    ADMUX |= BIT(MUX2);
    break;

    case A5:
    ADMUX |= BIT(MUX2)|BIT(MUX0);
    break;

    case A6:
    ADMUX |= BIT(MUX2)|BIT(MUX1);
    break;

    case A7:
    ADMUX |= BIT(MUX2)|BIT(MUX1)|BIT(MUX0);
    break;

    //case A8:
    //ADMUX |= BIT(MUX3);
    //break;
    }
  ADMUX |= BIT(REFS0);
 
  // set ADC control and status register A
  ADCSRA = 0;
 // float freq = 1/n;
  ADCSRA |= BIT(ADEN); // ADC enable
  ADCSRA |= BIT(ADIE); // ADC interrupt enable

  // ADPS0, ADPS1, ADPS2 - system clock to ADC input clock prescaler bits
  // smaller prescaler results in faster conversions, but too fast
  // reduces ADC resolution and accuracy
  ADCSRA |= BIT(ADPS2)|BIT(ADPS1)|BIT(ADPS0) ; // 64 prescaler (2nd slowest) 
  //Serial.println(micros()/1000000);   
    ADCSRA |= BIT(ADSC); // start ADC conversion
  
  // note: the interrupt will occur when ADC conversion is complete
  // int nothing=0;
  sei(); // enable interrupts
  int u;
  while(count!=200){
    u++;
  }

    
    
//Serial.print("sum_ADC:");
//Serial.println(sum_ADC);    
input = (float)sum_ADC/200; // average analog input n =100


  // note that the simple division of float below takes around 
  // 40 us compared to around 4 us for equivalent multiplication
  // -> avoid using float division if possible
  // -> use multiplication instead of division when possible
  // -> avoid putting float division in loops
//  voltage = input / 1023.0 * 5;
  voltage = input * ADC_to_V; // much faster than expression above
  //Serial.print("voltage: ");
 // Serial.println(micros());
  return voltage;


  
  }


double get_time_us()
{  
  float t;
  
  t = TCNT2/16.0e6*8;
  
  cli();
  t += t2_offset; // add offset to clock time
  sei();
  
  return t*1.0e6;
}

void steering_control(float yaw)
{
float ratio;
  pin_setup(3, OUTPUT);
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

 //pulse = ((0.5/90)*(angle_degrees-45)+1.25)*1000;
 ratio = angle_degrees/180;
 set_pwm(3,ratio , 0.02);


 
}



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////INTERUPTS///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//COMPARE MATCH TIMERS FOR OUR PWM FUNCTION
ISR(TIMER1_COMPA_vect)
// timer1 output compare match interrupt function
{
//   (count-- > value) ? PORTB |= BIT(1) : PORTB &= ~BIT(1);
//pin 9
  static float prev, after;
  
  PORTB |= BIT(1); //on for z us
  prev = TCNT1;
  while((TCNT1-prev) < z_9+10){  }
  PORTB &= ~BIT(1);
  after = TCNT1;
  
 // Serial.print("\tafter - prev = ");
 // Serial.println(after - prev);
}
ISR(TIMER2_COMPA_vect)
{
//pin 3
  static float prev, after;
  
  PORTD |= BIT(3); //on for z us
  prev = TCNT2;
  while((TCNT2 - prev) < z_3){  }
  PORTD &= ~BIT(3);
  after = TCNT2;
  
//  Serial.print("\tafter - prev = ");
//  Serial.println(after - prev);
}
  

ISR(ADC_vect)
// ADC conversion complete interrupt
// note:
// arduino time functions such as micros() and serial 
// communication can work in this interrupt function
{
  
  float input;
  //const float ADC_to_V = 1.0/1023.0*5;
  int adc1;
    if(count<200){
       count++;
       sum_ADC += ADC; 
       //Serial.println(count);
  ADCSRA |= BIT(ADSC); // start new ADC conversion that 
 // Serial.println(ADC);
 
 // Serial.print("count ");
  //Serial.println(count);
  //Serial.print("sum :");

   //Serial.println(sum_ADC);
    }
    else {
      //input = (float)sum_ADC / count; // average analog input
  
  
     
// Serial.println(micros()/1000000);
        
      }

  // read the ADC (10-bits) // 0 - 1023
  
}
