#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#define BIT(a) (1UL << (a))


//set variables
float t0, micro_t;
unsigned long volatile int sum_ADC=0;
volatile unsigned int Ninterrupts;
volatile int count=0;
volatile double t2_offset = 0.0;

bool portB[] = {0, 0, 0, 0, 0, 0}; //default all pins are switch off in terms of PWM
             //{8, 9, 10, 11, 12, 13};
bool portD[] = {0, 0, 0, 0, 0, 0, 0, 0}; //default all pins are switch off in terms of PWM
              //{0, 1, 2, 3, 4, 5, 6, 7}; 
float flag_counter = 0;
int on_limit[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};// gets immidetely overwritten
               //{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}
double period_s; //Microseconds
double period_counts; //Counts
static double ISR_period = 200;
double limit_period = 100; //Counts. initialize to 1000. user sets later.
int time1, time2;



//Function: create a pwm signal 
//Input: any digital pin; the duty cycle in microseconds
//Output: N/A
void pwm(int pin, int duty){

  //turn from microseconds to counts -> amount of counts that has to be ON
  int duty_counts = duty * 0.5; // 0.5 microseconds is 1 clock cycle with a prescaler of 8
  
  //turn count to number of ISR ticks
  // duty of signal in # of ISR calls is = duty clock counts / the number of ISR calls in 1 period
  int duty_ISR_count = duty_counts/(ISR_period);
  
  //find how many counts in is the OFF state, and thats when it'll be turned on
  on_limit[pin] = limit_period - duty_ISR_count;

}




//Function: set the period of the PWM in microseconds
//Input: period in microseconds
//Output: N/A
void set_period(double period){
 //intended to only be called once
 period_s = period*pow(10,-6); //period in seconds
 float period_of_ocr_call_us = period / limit_period; //period is split into 100 pieces / 100 ISR compare match calls
 float period_of_ocr_call_s = period_of_ocr_call_us *pow(10, -6); //  us to s
 ISR_period = period_of_ocr_call_s * 16e6/8; //8 is the prescaler & this finds the compare match value required
 
 for(int i = 0; i<=13; i++)
   on_limit[i] = limit_period +1; //put all the pins off

 pwm_setup_timer1(); 
}





//Function: setup timer1 for our pwm function 
//Input: N/A
//Output: N/A
void pwm_setup_timer1(){
        cli();
        TCCR1A = 0; // clear timer1 control register
        TCCR1B = 0;
        TCCR1B |= BIT(WGM12);
        TCCR1B |= BIT(CS11);//prescaler of 8 
          
        // initialize timer1
        TCNT1 = 0;

        // clear previous overflow interrupts so an overflow interrupt doesn't 
        // immediately occur when enabled below
        TIFR1 |= BIT(TOV1);
        OCR1A = ISR_period; //match_register calculated in the period setup function

        TIFR1 |= BIT(OCF1A);
        TIMSK1 |= BIT(OCIE1A);  

        sei();
        TCNT1 = 0;
        
}





//Function: get the time since the start in microseconds
//Input: N/A
//Output: time in microseconds since program began
double get_time_us()
{  
  float t;
  
  t = TCNT2/16.0e6*8;
  
  cli();
  t += t2_offset; // add offset to clock time
  sei();
  
  return t*1.0e6;
  
}





//Function: setup timer 1
//Input: N/A
//Output: N/A
void get_time_us_setup(){
//timer 2 is 8 bit so it counts to 255
cli(); 
TCCR2A = 0;
TCCR2B = 0;
TCCR2B |= BIT(CS11); //8 prescaler
TCNT2 = 0;  
TIFR2 |= BIT(TOV1); //clear prev overflow interrupts
TIMSK2 = BIT(TOIE1); //enable overflow interrupt
sei(); // enable interrupts
TCNT2 = 0;
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
        //DDRB logic 0 and PORTxn logic 0
        //ie pullup resistor off
        int oldSREG = SREG;
        cli();
        DDRB &= ~BIT(i);
        PORTB &= ~BIT(i);
        SREG = oldSREG;}
      else if (mode == INPUT_PULLUP){
        //DDRB logic 0 and PORTxn logic 1
        int oldSREG = SREG;
        cli();
        DDRB &= ~BIT(i);
        PORTB |= BIT(i);
        SREG = oldSREG;}
      else if(mode == OUTPUT){
        //DDRB logic 1
        int oldSREG = SREG;
        cli();
        DDRB |= BIT(i);
        SREG = oldSREG;
        }
  }
  
  //PORT D
  if(pin>=0 && pin<=7){
      if(mode == INPUT){
        //
        int oldSREG = SREG;
        cli();
        DDRD &= ~BIT(i);
        PORTB &= ~BIT(i);
        SREG = oldSREG;}
      else if (mode == INPUT_PULLUP){
        int oldSREG = SREG;
        cli();
        DDRD &= ~BIT(i);
        PORTD |= BIT(i);
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
float read_ADC_voltage_register(int channel,int n){
  cli();
  int i; 
  float input, voltage;
  const float ADC_to_V = 1.0/1023.0*5;
  Ninterrupts = n;
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
  float freq = 1/n;
  ADCSRA |= BIT(ADEN); // ADC enable
  ADCSRA |= BIT(ADIE); // ADC interrupt enable

  // ADPS0, ADPS1, ADPS2 - system clock to ADC input clock prescaler bits
  // smaller prescaler results in faster conversions, but too fast
  // reduces ADC resolution and accuracy
  ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2); // 128 prescaler (slowest)    
  ADCSRA |= BIT(ADSC); // start ADC conversion
  // note: the interrupt will occur when ADC conversion is complete
   int nothing=0;
  sei(); // enable interrupts
  int u;
  while(count!=n){
    u++;
  }


    


input = (float)sum_ADC/n; // average analog input

  voltage = input * ADC_to_V; // much faster than expression above
  return voltage;

  
  }





//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////INTERUPTS///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

//COMPARE MATCH TIMERS FOR OUR PWM FUNCTION
ISR(TIMER1_COMPA_vect)
{

  flag_counter++; 
  //check the limits and determine if pin should be on or off
  //check which PORT as well
  for(int i = 0; i<=13; i++){
    if(flag_counter == on_limit[i]){
      if(i>=8)
      PORTB |= BIT(i-8); //on
      
      if(i<8)
      PORTD |= BIT(i); //on 
      
  }
 }
  //reset flag counter after the period has completed
  if(flag_counter >= limit_period) {
    
    flag_counter = 0;
    for(int i = 0; i<=13; i++){
      if(i>=8)
         PORTB &= ~BIT(i-8); //off
      
      if(i<8)
         PORTD &= ~BIT(i); //off
      
    }
      
  }

}

ISR(ADC_vect)
// ADC conversion complete interrupt
// note:
// arduino time functions such as micros() and serial 
// communication can work in this interrupt function
{
  float input;
  const float ADC_to_V = 1.0/1023.0*5;
  int adc1;
    if(count<Ninterrupts){
  ADCSRA |= BIT(ADSC); // start new ADC conversion that 
  
  count++;

   sum_ADC += ADC; 
    }
    else {
      input = (float)sum_ADC / count; // average analog input
  
  
     
        
      }
}

ISR(TIMER2_OVF_vect)
{
  // increase offset by the roll time when the timer overflows
  t2_offset += 255.0/16.0e6*8;
  
}
