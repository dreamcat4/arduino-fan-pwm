//// begin: convert .ino to .cpp
//#include <Arduino.h>
//void doEncoder();
//void clicked();
//// end: convert .ino to .cpp

// Source: "Arduino Powered Smart Fan Controller" - Barnesian
//for wiring, see: https://barnesian.com/arduino-powered-smart-fan-controller/

#include <OneWire.h>
#include <DallasTemperature.h>
#include "PID_v1.h"
#include <LiquidCrystal.h>
 
//Definitions
//#define FAN 9           // PWM output pin for fan
#define ONE_WIRE_BUS 8  // Temperature Input is on Pin 2 of the Dallas ds18b20
#define click 3         //Rotary Encoder Click
//#define encoder0PinA  2 //Rotary Encoder Pin A
//#define encoder0PinB  4 //Rotary Encoder Pin B
#define CRITICAL 50.00  //Critical temperature to ignore PID and turn on fans

volatile unsigned int encoder0Pos = 0;  //Encoder value for ISR


/* Analog Read to LED
 * ------------------ 
 *
 * for wiring, see: https://www.arduino.cc/en/tutorial/potentiometer
 * 
 * Basically: (pin1) +5v --- WIPER --- (pin2) ANALOG_PIN_A2 --- WIPER --- (pin3) GND
 * 
 *
 * turns on and off a light emitting diode(LED) connected to digital  
 * pin 13. The amount of time the LED will be on and off depends on
 * the value obtained by analogRead(). In the easiest case we connect
 * a potentiometer to analog pin 2.
 *
 * Created 1 December 2005
 * copyleft 2005 DojoDave <http://www.0j0.org>
 * http://arduino.berlios.de
 *
 */

int potPin = 2;    // select the input pin for the potentiometer
int pot0_val = 0;       // variable to store the value coming from the sensor



// Source: http://www.gammon.com.au/forum/?id=11504
// Example of modulating a 38 kHz frequency duty cycle by reading a potentiometer
// Author: Nick Gammon
// Date: 24 September 2012


//pin_type  REG    pkg_pin Digital_Pin
//========  ====   ======= ===========
//
//Timer 0
//=======
//input     T0     pin  6  (D4)
//output    OC0A   pin 12  (D6)
//output    OC0B   pin 11  (D5)
//
//Timer 1
//=======
//input     T1     pin 11  (D5)
//output    OC1A   pin 15  (D9)
//output    OC1B   pin 16  (D10)
//
//Timer 2
//=======
//output    OC2A   pin 17  (D11)
//output    OC2B   pin  5  (D3)


//const byte POTENTIOMETER = A0;
//const byte LED = 10;  // Timer 1 "B" output: OC1B
//#define FAN 9           // PWM output pin for fan
#define FAN 10           // PWM output pin for fan

// Clock frequency divided by 38 kHz frequency desired
const long timer1_OCR1A_Setting = F_CPU / 38000L;

//void setup() 
// {
//  pinMode (LED, OUTPUT);
//
//  // set up Timer 1 - gives us 38.005 kHz 
//  // Fast PWM top at OCR1A
//  TCCR1A = bit (WGM10) | bit (WGM11) | bit (COM1B1); // fast PWM, clear OC1B on compare
//  TCCR1B = bit (WGM12) | bit (WGM13) | bit (CS10);   // fast PWM, no prescaler
//  OCR1A =  timer1_OCR1A_Setting - 1;                 // zero relative  
//  }  // end of setup

//void loop()
//  {
//  // alter Timer 1 duty cycle in accordance with pot reading
//  OCR1B = (((long) (analogRead (POTENTIOMETER) + 1) * timer1_OCR1A_Setting) / 1024L) - 1;
//  
//  // do other stuff here
//  }
 



 
// LiquidCrystal lcd(12, 11, 13, 5,6,7);  //set up LCD
 
//Setup Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
 
//Setup PID
double setPoint, sensor0_temp, Output;                                          //I/O for PID
double aggKp=40, aggKi=2, aggKd=10;                                      //original: aggKp=4, aggKi=0.2, aggKd=1, Aggressive Turning,50,20,20
double consKp=20, consKi=1, consKd=5;                                    //original consKp=1, consKi=0.05, consKd=0.25, Conservative Turning,20,10,10
PID myPID(&sensor0_temp, &Output, &setPoint, consKp, consKi, consKd, REVERSE);  //Initialize PID
 
//interface
int timeCounter;
void setup()
{  

  //Setup Pins
  pinMode(FAN, OUTPUT);                   // Output for fan speed, 0 to 255
  pinMode(click, INPUT);                  // Click button is an input
  pinMode(potPin, INPUT);                 // Click potPin is an input


  // set up Timer 1 - gives us 38.005 kHz 
  // Fast PWM top at OCR1A
  TCCR1A = bit (WGM10) | bit (WGM11) | bit (COM1B1); // fast PWM, clear OC1B on compare
  TCCR1B = bit (WGM12) | bit (WGM13) | bit (CS10);   // fast PWM, no prescaler
//  TCCR1B = bit (WGM10) | bit (WGM11) | bit (COM1B1);
  OCR1A =  timer1_OCR1A_Setting - 1;                 // zero relative  
//  OCR1B =  timer1_OCR1A_Setting - 1;                 // zero relative  


// start serial port for temperature readings
  Serial.begin(9600);
  Serial.println("Start");
  
  //Temperature Setup
  sensors.begin();                    //Start Library
  sensors.requestTemperatures();      // Send the command to get temperatures
  sensor0_temp = sensors.getTempCByIndex(0); //Set Input to Current Temperature

  setPoint = 28;                      //Inintialize desired Temperature in Deg C

  double temp0Min = 25;
  double temp0Max = 50;
  double temp0Tgt = setPoint;

//  double temp1Min = 25;
//  double temp1Max = 50;
//  double temp1Tgt = setPoint;
//  double temp2Min = 25;
//  double temp2Max = 50;
//  double temp2Tgt = setPoint;
//  double temp3Min = 25;
//  double temp3Max = 50;
//  double temp3Tgt = setPoint;

/*
pid parameters
==================
p = proportional

p = proportional band = %pb = 100/pb * error
  pb = 100/p

Kp = gain = p = higher the faster the proportional response
 for example a gain of 18x means the output control variable (cv) will be 18*error

wheras proportional band = pb or kp = 100/p
then a lower value is higher gain... 1/18th as a % = 5.55%... so cv=100*error/pb
====================================================

i = integral = final little 'push' to nudge the temperature onto target

integral notices if the error value remains uncorrected
this occurs once the error gets close enough to the sepoint, 
that the proportional value is no longer strong enough to give a sufficient
positive enough value to overcome the inertia threshold for the last little bit
for example the static friction in a movement system, or other signal inertia

Ki = the integral constant = integral Gain
however very often there is no seperate term for the Integral Gain,
instead it's all rolled into the shared common gain 'K', when  Kp = Ki

======================================================

however there is usually the 'inegral remembering time period'

integral reset = Tau_i

i = reset, can be defined in:
repeats per sec = hz
seconds per repeat
repeats per min
minutes per repeat

the integral = sum of area under curve
Ki, or Ti. where Ki = 1/Ti

what is Tau_i?
================
it's the term that we successively divide the Ki by
for example:

Ki/Tau_i * e
we need a working example of this
but basically the integral keeps forgetting the older terms each sample

the integral is only remembered since the last time period
it's like a sliding window
for example an integral's reset period of '6rpm'
means that it only remembers the last 10 seconds worth of error

integral wind up
===================
occurs when the output motor is saturated
and cannot spin faster than a given rpm

this causes the error (e) to remain high, despite the output driving value being high
so then the integral keeps adding up

integral clamping
===================

to combat this issue, a special loop will impose limit on the integral value
this occurs whenever the output value exceeds the threshold, then we will clamp
once clamping is triggered, then the error term for the integral (Ei) will be set to 0
so long as the sign +- of the error term is the same as the sign of the output
which clears the integral value, and makes it 0, until clamping is released again
which occurs if there is no longer saturation, or if there is overshoot and positional error sign changes

integral clamping limit should be set lower than the maximum rpm of the output motor when saturated


====================================================
d = derivative = sensitive to noise spikes (high dt), which induce unwanted oscillations

d = d/dt = dpv/dt
the rate of change of the perceived value, over time
so this is how fast the rate the system is currently changing it's temperature at
this is governed by the responsiveness of the system



sp = set point = target temp
pv = process variable / perceived value = current temp
cv = control variable = mv (manupulated variable) = output to heater
e = error = (set point - perceived value) = distance to target


high freq noise - low pass cutoff filter
++++++++++++++++++++++++++++++++++++++++++++++++++++++

d/dt derivative is laeger for hf noise spikes
so we need to use a low pass filter on the derivative fumctiom

tuning the filter freq is an important variable

laplace domain tranfer function
================================
from 'undersstanding pid control part 3' video, at 10m45s

to incorporate both the derivative + low pass filter
we can instead create a negative feedback loop with the integral

this is more computationally effecient transfer function
combining the 2 blocks together.

*/
   
  //PID Setup
  myPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
  

//  pinMode(encoder0PinA, INPUT); 
//  digitalWrite(encoder0PinA, HIGH);       // Turn on pullup resistor
//  pinMode(encoder0PinB, INPUT); 
//  digitalWrite(encoder0PinB, HIGH);       // Turn on pullup resistor
 
  //Set up Interupts
//  attachInterrupt(1, clicked, RISING);    // Click button on interrupt 1 - pin 3
//  attachInterrupt(0, doEncoder, CHANGE);  // Encoder pin on interrupt 0 - pin 2
  
  //interface
   
  timeCounter=0;
    
  // //Setup LCD 16x2 and display startup message
  // lcd.begin(16, 2);
  // lcd.print("  Smart   Fan");
  // lcd.setCursor(0,1);
  // lcd.print("  Starting Up");
  // delay(1000);
  // lcd.clear();
}
 
void loop()
{
  timeCounter++;

  pot0_val = analogRead(potPin);    // read the value from the sensor

    
  //Get temperature and give it to the PID input
  sensors.requestTemperatures();
  sensor0_temp=sensors.getTempCByIndex(0);
   
  // //print out info to LCD
  // lcd.setCursor(1,0);
  // lcd.print("Temp:");
  // lcd.print((int)sensor0_temp);
  // lcd.setCursor(9,0);
  // lcd.print("RPM:");
  // lcd.print((int)Output*4.7059);
  // lcd.setCursor(1,1);
  // lcd.print("Set:");
  // lcd.print((int)setPoint);
   
  //Compute PID value
  double gap = abs(setPoint-sensor0_temp); //distance away from setpoint
  if(gap < 1)
  {  
    //Close to setPoint, be conservative
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //Far from setPoint, be aggresive
     myPID.SetTunings(aggKp, aggKi, aggKd);
  } 
  myPID.Compute();
//  Serial.print(timeCounter);

  Serial.print("    ");
  Serial.print("sensor0_temp=");
  Serial.print(sensor0_temp);

  Serial.print(",   ");
  Serial.print("pot0_val=");
  Serial.print(pot0_val);

  Serial.print(",   ");
  Serial.print("RPM=");
  Serial.print((int)Output*4.7059);

  Serial.print(",   ");
  Serial.print("pidOutput=");
  Serial.println(Output);
 
//  //Write PID output to fan if not critical
//  if (sensor0_temp < CRITICAL)
//    analogWrite(FAN,Output);
//  else
//    analogWrite(FAN,255);


  // alter Timer 1 duty cycle in accordance with pot reading
  OCR1B = (((long) (pot0_val + 1) * timer1_OCR1A_Setting) / 1024L) - 1;

//  OCR1A = (((long) (pot0_val + 1) * timer1_OCR1A_Setting) / 1024L) - 1;



}
 
//void doEncoder()
//{
//  //pinA and pinB are both high or both low, spinning forward, otherwise it's spinning backwards
//  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
//  {
//    encoder0Pos++;
//  }
//  else
//  {
//    encoder0Pos--;
//  }
////  Serial.println (encoder0Pos, DEC);  //Print out encoder value to Serial
//  setPoint=encoder0Pos;
//}
 
void clicked()
{
  // //For interface
  // lcd.clear();
  // lcd.print("clicked!");

  Serial.print("Clicked!");
  Serial.print(" ");
  delay(1000);
}
