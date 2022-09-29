#include "car_control.h"
#include "microcontroller_functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//set variables
float z_3 = 50;
float z_9 = 50;
float t0;
long volatile int sum_ADC=0;
volatile int count=0;
volatile unsigned int Ninterrupts;
//Function: create a pwm signal 
//Input: pin 3, 11, 9, 10; the duty cycle, and the period
//Output: N/A
void set_pwm(int pin, float duty, float period){
  //we don't want to mess around with timer 0, so pins 5 and 6 will not be used
    
  long cycles=0; 
  if(period > 0) {
    cycles = (F_CPU * period) / 2000000;
    //to do: check which pin is A and which pin is B 
    //in order to be able to use all pwm pins at the same time
    if(pin == 9 ){
        z_9 = duty;
        cli();
        TCCR1A = 0;
        TCCR1B = 0;
        TCCR1B |= BIT(WGM12);
        TCCR1B |= BIT(CS11) | BIT(CS10); //prescaler 64
        //calculate period 
        //period in seconds = match_register * 64 /16e6
        unsigned int match_register = period *(16*pow(10, 6)) / 64;
        OCR1A = match_register; //match_register;
        Serial.print(OCR1A);
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
        unsigned int match_register = period *(16*pow(10, 6)) / 64;
        Serial.print(match_register);
        OCR2A = match_register;
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





float read_ADC_voltage_regsiter(int channel,int n){
 
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
  while(count!=200){
    u++;
  }

    
    
Serial.print("sum_ADC:");
Serial.println(sum_ADC);
input = (float)sum_ADC/n; // average analog input


  // note that the simple division of float below takes around 
  // 40 us compared to around 4 us for equivalent multiplication
  // -> avoid using float division if possible
  // -> use multiplication instead of division when possible
  // -> avoid putting float division in loops
//  voltage = input / 1023.0 * 5;
  voltage = input * ADC_to_V; // much faster than expression above
  Serial.print("voltage: ");
  Serial.println(voltage);
  return voltage;


  
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
  while((TCNT1 - prev) < z_9){  }
  PORTB &= ~BIT(1);
  after = TCNT1;
  
//  Serial.print("\tafter - prev = ");
//  Serial.println(after - prev);
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
  const float ADC_to_V = 1.0/1023.0*5;
  int adc1;
    if(count<Ninterrupts){
  ADCSRA |= BIT(ADSC); // start new ADC conversion that 
  
  count++;
 // Serial.print("count ");
  //Serial.println(count);
  //Serial.print("sum :");
   sum_ADC += ADC; 
   //Serial.println(sum_ADC);
    }
    else {
      input = (float)sum_ADC / count; // average analog input
  
  
     
 // Serial.print("\nADC complete interrupt, i = ");
        
      }

  // read the ADC (10-bits) // 0 - 1023
  
}
