
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
// #include <LiquidCrystal.h>

// not sure why this isn't defined
#define uint  uint16_t
#define ulong uint32_t
 
//Definitions
//#define FAN 9           // PWM output pin for fan
#define ONE_WIRE_BUS 8  // Temperature Input is on Pin 2 of the Dallas ds18b20
// #define click 3         //Rotary Encoder Click
//#define encoder0PinA  2 //Rotary Encoder Pin A
//#define encoder0PinB  4 //Rotary Encoder Pin B
#define CRITICAL 50.00  //Critical temperature to ignore PID and turn on fans


// #define FAN0_SENSE 2
// #define FAN1_SENSE 3
// #define FAN2_SENSE 0

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
   ulong tach, last_tach;
   double rpm_min, rpm_max, rpm_target;
   double rpm;
   uint pad;
};

fan fan0 = { 2, 10, 0, 0, 200, 1000, 2300, 0};
fan fan1 = { 3, 12, 0, 0, 200, 2000, 1000, 0};


struct pwm
{
  float pwm_min, pwm_max, pwm_target;
  float pwm;
  uint pad;
};

struct pid
{
  double setpoint;
  double kp, ki, kd;
};



 

// arduino uno atmega328p only has 3 fast timer registers
timer timer0, timer1, timer2;
pwm pwm0, pwm1;



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
 

// float fan0_overide = 2600;
float fan0_overide = 5000;

float rpm_threshold_near_far = 800;
 
// LiquidCrystal lcd(12, 11, 13, 5,6,7);  //set up LCD
 
//Setup Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

double setPoint, sensor0_temp, Output;                                          //I/O for PID
double innerS, outerS;

// don't output 0.00 for the pwm duty cyle!
// value pwm minimum value is slightly above zero
double innerS_min = 0002.00;

// i believe it's set to 1024 , for 100% pwm
// double innerS_max = 1000.00;
double innerS_max = 1022.00;
// double innerS_max = 1023.00;
// double innerS_max = 1024.00;
// double innerS_max = 2048.00;



// Setup PID

// inner pid = ip_, outer pid = op
// + conservative = _slow, aggressive = _fast

pid ip_slow = {  5.0, 03.0, 0.5 };
pid ip_fast = {  5.0, 03.0, 0.5 };
// pid ip_fast = {  8.0, 00.0, 2.0 };

// pid ip_slow = { 2, 0.1, 0.5 };
// pid ip_fast = { 4, 0.2, 1.0 };

// pid ip_slow2 = { 20, 10, 10 };
// pid ip_fast2 = { 50, 50, 20 };

PID innerPID(&fan0.rpm, &innerS, &fan0.rpm_target, ip_slow.kp, ip_slow.ki, ip_slow.kd, DIRECT);

pid op_slow = { 20, 01, 05 };
pid op_fast = { 40, 02, 10 };
// pid op_slow2 = { 20, 10, 10 };
// pid op_fast2 = { 50, 50, 20 };
PID outerPID(&sensor0_temp, &outerS, &setPoint, op_slow.kp, op_slow.ki, op_slow.kd, REVERSE);


// fan tachometer
// arduino uno atmega 328p can only support max 2 digital pin interrupts!!

ulong ms_ellapsed;
// ulong fan0_tach, fan1_tach;
// float fan0_rpm, fan1_rpm;
// float fan0_rpm_tgt, fan1_rpm_tgt;

void fan0_tick()
{
  // Serial.println("    fan0_tick()");
  // fan0_tach++;
  fan0.tach++;
}

void fan1_tick()
{
  fan1.tach++;
}



void clear_tachs()
{
  fan0.tach = 0;
  fan1.tach = 0;
}

unsigned long start, stop;

//interface
int loopCounter;
void setup()
{  
  // clear_tachs();


  //Setup Pins
  pinMode(fan0.pin_pwm, OUTPUT);                   // Output for fan speed, 0 to 255
  pinMode(fan1.pin_pwm, OUTPUT);                   // Output for fan speed, 0 to 255


  pinMode(fan0.pin_tach, INPUT);
  // pinMode(fan0.pin_tach, INPUT_PULLUP);
  // pinMode(fan0.pin_tach, INPUT_PULLDOWN);

  // pinMode(fan1.pin_tach, INPUT_PULLUP);

  // setup interupt callbacks
  // arduino uno atmega 328p can only support max 2 digital pin interrupts!!
  attachInterrupt(digitalPinToInterrupt(fan0.pin_tach), fan0_tick, RISING);
  // attachInterrupt(digitalPinToInterrupt(fan1.tach_pin), fan1_tick, RISING);



  // arduino uno atmega 328p can only support max 2 digital pin interrupts!!
  // pinMode(FAN2_SENSE, INPUT);

  // set up Timer 1 - gives us 38.005 kHz 
  // Fast PWM top at OCR1A
  TCCR1A = bit (WGM10) | bit (WGM11) | bit (COM1B1); // fast PWM, clear OC1B on compare
  TCCR1B = bit (WGM12) | bit (WGM13) | bit (CS10);   // fast PWM, no prescaler
//  TCCR1B = bit (WGM10) | bit (WGM11) | bit (COM1B1);
  OCR1A =  timer1_OCR1A_Setting - 1;                 // zero relative  
//  OCR1B =  timer1_OCR1A_Setting - 1;                 // zero relative  


  // start serial port for temperature readings

  // Serial.begin(9600);
  Serial.begin(19200);
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



   
  //PID Setup
  innerPID.SetMode(AUTOMATIC);
  innerPID.SetOutputLimits(innerS_min, innerS_max);

  outerPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
  

//  pinMode(encoder0PinA, INPUT); 
//  digitalWrite(encoder0PinA, HIGH);       // Turn on pullup resistor
//  pinMode(encoder0PinB, INPUT); 
//  digitalWrite(encoder0PinB, HIGH);       // Turn on pullup resistor
 


   
  loopCounter=0;

  // ater requesting a temperature sample from a dallas ds18b20, we have to wait for the sensor to respond back
  sensors.setWaitForConversion(false);  // make it async
  sensors.requestTemperatures();
}

void outer_loop()
{
  Serial.println("");
  Serial.println("outer_loop()");

  // input  = temp
  // output = fan rpm value
  // output = relay, OTP over temperature timeout period exceeded

  // // get the potentiometer input value
  // pot0_val = analogRead(potPin);

  // now get the temperature
  sensor0_temp=sensors.getTempCByIndex(0);

  // Request the next temperature conversion in time for the next outer loop. non-blocking / async
  sensors.requestTemperatures();

  //Compute PID value
  double gap = abs(setPoint-sensor0_temp); //distance away from setpoint
  if(gap < 1)
  {  
    //Close to setPoint, be conservative
    outerPID.SetTunings(op_slow.kp, op_slow.ki, op_slow.kd);
  }
  else
  {
     //Far from setPoint, be aggresive
     outerPID.SetTunings(op_fast.kp, op_fast.ki, op_fast.kd);
  } 

  outerPID.Compute();


  Serial.print("  ");
  Serial.print("pot0_val=");
  Serial.print(pot0_val);

  Serial.print("    ");
  Serial.print("sensor0_temp=");
  Serial.print(sensor0_temp);


  Serial.print(",   ");
  Serial.print("pidOutput=");
  Serial.println(Output);

  Serial.println("");


  // fan0.rpm_target = fan0_overide;
}


void inner_loop()
{
  Serial.println("  inner_loop()");

  // input  = current fan rpm
  // output = desired fan rpm
  // output = relay, fan tach failed


  clear_tachs();
  // delay(5);
  // clear_tachs();
  // delay(5);
  
  start = millis();       
  delay(1000);
  stop  = millis();
  ms_ellapsed = stop - start;

  fan0.last_tach = fan0.tach;


  Serial.print("     ");
  Serial.print("ms_ellapsed=");
  Serial.print(ms_ellapsed); 

  Serial.print(",    ");
  Serial.print("fan0.last_tach=");
  Serial.print(fan0.last_tach);

  // fan0_rpm = fan0_tach / 4;

  fan0.rpm = (fan0.last_tach * 60 * 1000) / (2 * ms_ellapsed);
  // fan1_rpm = (fan1_last_tach * 60 * 1000) / ms_ellapsed;
  // clear_tachs();



  Serial.print("     ");
  Serial.print("fan0.rpm=");
  Serial.print(fan0.rpm);

  Serial.print("     ");
  Serial.print("fan0.rpm_target=");
  Serial.print(fan0.rpm_target);

  // Serial.print(",   ");
  // Serial.print("fan1_rpm=");
  // Serial.print(fan1_rpm);



  // Compute PID value
  double gap = abs(fan0.rpm_target - fan0.rpm); //distance away from setpoint

  Serial.print("     ");
  Serial.print("gap=");
  Serial.print(gap);

  if(gap < rpm_threshold_near_far)
  {  
    //Close to setPoint, be conservative
    innerPID.SetTunings(ip_slow.kp, ip_slow.ki, ip_slow.kd);
  }
  else
  {
     //Far from setPoint, be aggresive
     innerPID.SetTunings(ip_fast.kp, ip_fast.ki, ip_fast.kd);
  } 

  innerPID.Compute();

  // // don't output 0.00 for the pwm duty cyle!
  // if (innerS < innerS_min)
  //   innerS = innerS_min;

  Serial.print(",   ");
  Serial.print("innerS (PID)=");
  Serial.println(innerS);

//  //Write PID output to fan if not critical
//  if (sensor0_temp < CRITICAL)
//    analogWrite(FAN,Output);
//  else
//    analogWrite(FAN,255);


  // alter Timer 1 duty cycle in accordance with pot reading
  // OCR1B = (((long) (pot0_val + 1) * timer1_OCR1A_Setting) / 1024L) - 1;
  OCR1B = (((long) (innerS + 1) * timer1_OCR1A_Setting) / 1024L) - 1;

//  OCR1A = (((long) (pot0_val + 1) * timer1_OCR1A_Setting) / 1024L) - 1;
  // Serial.println("loop6");


  loopCounter++;

  // Serial.println(loopCounter);
  Serial.println("");

}

void loop()
{
  inner_loop();

  if (loopCounter % pid_loop_outer_inner_ratio == 0)
    outer_loop();
}



