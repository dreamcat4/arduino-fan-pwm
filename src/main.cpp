
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "PID_v1.h"
// #include <LiquidCrystal.h>

// not sure why this isn't defined
#define uint uint8_t

 
//Definitions
//#define FAN 9           // PWM output pin for fan
#define ONE_WIRE_BUS 8  // Temperature Input is on Pin 2 of the Dallas ds18b20
#define click 3         //Rotary Encoder Click
//#define encoder0PinA  2 //Rotary Encoder Pin A
//#define encoder0PinB  4 //Rotary Encoder Pin B
#define CRITICAL 50.00  //Critical temperature to ignore PID and turn on fans

volatile unsigned int encoder0Pos = 0;  //Encoder value for ISR


// Analog Read to LED
// for wiring, see: https://www.arduino.cc/en/tutorial/potentiometer
// Basically: (pin1) +5v --- WIPER --- (pin2) ANALOG_PIN_A2 --- WIPER --- (pin3) GND
int potPin = 2;    // select the input pin for the potentiometer
int pot0_val = 0;       // variable to store the value coming from the sensor


struct timer
{
};

struct fan
{
   uint pin_tach, pin_pwm;
   uint rpm_min, rpm_max, rpm_target;
   uint rpm;
   uint pad;
};

struct pwm
{
  uint pwm_min, pwm_max, pwm_target;
  uint pwm;
  uint pad;
};

struct pid
{
  double setpoint;

  // Close to setPoint, be conservative
  double consKp, consKi, consKd;

  // Far from setPoint, be aggresive
  double aggKp, aggKi, aggKd;
};



// double setPoint, sensor0_temp, Output;                                          //I/O for PID
// double aggKp=40, aggKi=2, aggKd=10;                                      //original: aggKp=4, aggKi=0.2, aggKd=1, Aggressive Turning,50,20,20
// double consKp=20, consKi=1, consKd=5;                                    //original consKp=1, consKi=0.05, consKd=0.25, Conservative Turning,20,10,10
// PID myPID(&sensor0_temp, &Output, &setPoint, consKp, consKi, consKd, REVERSE);  //Initialize PID
 

// arduino uno atmega328p only has 3 fast timer registers
timer timer0, timer1, timer2;
fan fan1, fan2, fan3;
pwm pwm1, pwm2, pwm3;

pid inner_pid0;


// uint pid_loop_inner_freq = 100;
uint pid_loop_outer_inner_ratio = 7;



// PWM Modulating a 38 kHz frequency duty cycle
// Source: http://www.gammon.com.au/forum/?id=11504

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
int loopCounter;
void setup()
{  
  inner_pid0.aggKp = 40;
  inner_pid0.aggKi = 02;
  inner_pid0.aggKd = 10;
  inner_pid0.consKp = 20;
  inner_pid0.consKi = 20;
  inner_pid0.consKd = 20;


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
   
  loopCounter=0;

  // ater requesting a temperature sample from a dallas ds18b20, we have to wait for the sensor to respond back
  sensors.setWaitForConversion(false);  // make it async
  sensors.requestTemperatures();
}

void outer_loop()
{
  Serial.println("outer_loop()");

  // input  = temp
  // output = fan rpm value
  // output = relay, OTP over temperature timeout period exceeded

  // get the potentiometer input value
  pot0_val = analogRead(potPin);

  // now get the temperature
  sensor0_temp=sensors.getTempCByIndex(0);

  // Request the next temperature conversion in time for the next outer loop. non-blocking / async
  sensors.requestTemperatures();

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


  Serial.print(",   ");
  Serial.print("pot0_val=");
  Serial.print(pot0_val);

  Serial.print("    ");
  Serial.print("sensor0_temp=");
  Serial.print(sensor0_temp);


  Serial.print(",   ");
  Serial.print("pidOutput=");
  Serial.println(Output);
}


void inner_loop()
{
  Serial.println("inner_loop()");

  // input  = fan rpm
  // output = persent % pwm duty cycle
  // output = relay, fan tach failed

  // //Compute PID value
  // double gap = abs(setPoint-sensor0_temp); //distance away from setpoint
  // if(gap < 1)
  // {  
  //   //Close to setPoint, be conservative
  //   myPID.SetTunings(consKp, consKi, consKd);
  // }
  // else
  // {
  //    //Far from setPoint, be aggresive
  //    myPID.SetTunings(aggKp, aggKi, aggKd);
  // } 

  // myPID.Compute();


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
  // Serial.println("loop6");


  loopCounter++;
}

void loop()
{
  inner_loop();

  if (pid_loop_outer_inner_ratio % loopCounter == 0)
    outer_loop();
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
 
// void clicked()
// {
//   // //For interface
//   // lcd.clear();
//   // lcd.print("clicked!");

//   Serial.print("Clicked!");
//   Serial.print(" ");
//   delay(1000);
// }
