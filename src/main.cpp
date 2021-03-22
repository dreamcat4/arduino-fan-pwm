
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
// #include <PID_AutoTune_v0.h>
// #include <EnableInterrupt.h>

// not sure why this isn't defined
#define uint  uint16_t
#define ulong uint32_t

// Analog Read Potentiometer Value
// for wiring, see: https://www.arduino.cc/en/tutorial/potentiometer
// Basically: (pin1) +5v --- WIPER --- (pin2) ANALOG_PIN_A2 --- WIPER --- (pin3) GND
// int potPin = 2;    // select the input pin for the potentiometer
// int pot0_val = 0;       // variable to store the value coming from the sensor
// uint pid_loop_inner_freq = 10;
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

// Clock frequency divided by 38 kHz frequency desired
const long timer1_OCR1A_Setting = F_CPU / 38000L;


// don't output 0.00 for the 0% pwm duty cyle!
// because it causes the arduino timer to then output +5v continuus
// on the arduino uno atmega 328p: '2.00' seems to be the lowest
// possible valuem that still produces a valid pwm signal
double innerS_min = 0002.00;

// don't output 1024, for 100% pwm duty cycle!
// because it causes the arduino timer to then output 0v continuus
// on the arduino uno atmega 328p: '1023.00' seems to be the highest
// possible value, that still produces a valid pwm signal
double innerS_max = 1023.00;

// recude this value to reduce fan overshoot
// double innerS_max = 900.00;

double tach_threshold_near_mid = 0.20;
// double tach_threshold_mid_fine = 0.05;

// double tach_threshold_near_mid = 0.160;
double tach_threshold_mid_fine = 0.040;

uint near_overshoots = 0;
uint max_near_overshoots = 3;

double innerS_max_change = 0.4;
double innerS_max_change_abs = innerS_max * innerS_max_change;

struct timer
{
};
// arduino uno atmega328p only has 3 fast timer registers
timer timer0, timer1, timer2;

// a dallas ds18b20 temperature object
struct temp
{
  float temp_min, temp_max, temp_target;
  float temp;
  uint pad;
};
temp temp0, temp1;

// pid object - a single set of pid configuration values
struct pid
{
  double setpoint;
  double kp, ki, kd;
};

struct fan
{
   uint pin_tach, pin_pwm;
   double rpm_min, rpm_max, rpm_target, pwm_max_change_abs;
   uint near_overshoots;
   double tach_min, tach_max, tach_target;
   double rpm, tach, last_tach, pwm, pwm_last;
   uint pad;
};


// example - how to declare a fan instance
// ========================================
// fan FAN_NAME = { arduino_pin_tach, arduino_pin_pwm, min_rpm, max_rpm, default_rpm, \
                    pwm_max_change_per_cycle, num_fine_seek_operations_until_settling }

fan fan0 = { 2, 10, 900, 2600, 1500, innerS_max_change_abs, max_near_overshoots };
fan fan1 = { 3, 12, 200, 2000, 1100, innerS_max_change_abs, max_near_overshoots };

// fan fans[] = { fan0, fan1 };
fan fans[] = { fan0 };
uint num_fans = sizeof(fans);

// Setup Dallas DS18b20 Temperature Sensor
// BTW i2c is on pin 2 (middle pin) of the Dallas ds18b20
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

double setPoint, sensor0_temp, Output;
double innerS, innerS_last;
double outerS, outerS_last;

uint innerS_delay = 1700;
// uint innerS_delay = 1000;

// inner pid = ip_, outer pid = op
// aggressive=_fast, medium=_mid, conservative=_fine

// original
// pid ip_fine_orig = { 20, 10, 10 };
// pid ip_fast_orig = { 50, 50, 20 };

// for rpm
// pid ip_fast = {  2.0, 00.5, 0.5 };
// pid ip_fast = { 4, 0.2, 1.0 };
// pid ip_fast = {  8.0, 00.0, 2.0 };

// pid ip_fine = {  2.7, 00.8, 0.6 };
// pid ip_fine = { 2, 0.1, 0.5 };

// for tach
// pid ip_fast = {  70, 40, 40 };
// pid ip_fast = {  30, 20, 20 };
pid ip_fast = {  140, 80, 80 };

pid ip_mid  = {  10, 10, 10 };

// pid ip_fine = {  27, 08.8, 6.6 };
// pid ip_fine = {  170, 120, 110 };
// pid ip_fine = {  70, 40, 40 };
// pid ip_fine = {  20, 10, 10 };
pid ip_fine = {  03, 03, 03 };



// PID innerPID(&fan0.rpm, &innerS, &fan0.rpm_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);
// PID innerPID(&fan0.last_tach, &innerS, &fan0.tach_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);
PID innerPID(&fan0.last_tach, &fan0.pwm, &fan0.tach_target, ip_fine.kp, ip_fine.ki, ip_fine.kd, DIRECT);

pid op_fast = { 40, 2.0, 10 };
pid op_med  = { 20, 1.0, 05 };
pid op_slow = { 10, 0.5, 02 };

// pid op_slow2 = { 20, 10, 10 };
// pid op_fast2 = { 50, 50, 20 };
PID outerPID(&sensor0_temp, &outerS, &setPoint, op_slow.kp, op_slow.ki, op_slow.kd, REVERSE);

// for inner pid loop
ulong ms_ellapsed;

// fan tachometer
// arduino uno atmega 328p can only support max 2 digital pin interrupts!!
void fan0_tick()   { fan0.tach++; }
void fan1_tick()   { fan1.tach++; }
void clear_tachs() { fan0.tach = 0; fan1.tach = 0; }

double tach_to_rpm(uint tach, uint ms_ellapsed)
{
   return ((double)tach * 60 * 1000) / (2 * ms_ellapsed);
}

uint rpm_to_tach(double rpm, uint ms_ellapsed)
{
  // double tach_exact = (rpm * 2 * (double)ms_ellapsed) / (60 * 1000);
  double tach_exact = (rpm * 2 / 60) * ms_ellapsed / 1000;

  // round to nearest whole integer
  return (uint)round(tach_exact);
}

void fan_instance_autofill_tachs(fan &fan_instance)
{
  fan_instance.tach_target = rpm_to_tach(fan_instance.rpm_target, innerS_delay);
  fan_instance.tach_min    = rpm_to_tach(fan_instance.rpm_min,    innerS_delay);
  fan_instance.tach_max    = rpm_to_tach(fan_instance.rpm_max,    innerS_delay);
}

void fan_instance_moderate_pwm(double gap, fan &fan_instance)
{
  if( abs(fan_instance.pwm - fan_instance.pwm_last) > fan_instance.pwm_max_change_abs )
  {
    if( fan_instance.pwm > fan_instance.pwm_last)
    {
      fan_instance.pwm = fan_instance.pwm_last + fan_instance.pwm_max_change_abs;
    }
    else
    {
      fan_instance.pwm = fan_instance.pwm_last - fan_instance.pwm_max_change_abs;       
    }
  }

  if(abs(gap) > tach_threshold_mid_fine)
  {
    fan_instance.pwm_last = fan_instance.pwm;
    fan_instance.near_overshoots = 0;
  }
  else if( (gap < 0) && (fan_instance.near_overshoots < max_near_overshoots) )
  {
    fan_instance.pwm_last = fan_instance.pwm;
    fan_instance.near_overshoots++;
  }
}

unsigned long start, stop;
int loopCounter;

void setup()
{  
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

  // set up Timer 1 - gives us 38.005 kHz 
  // Fast PWM top at OCR1A
  TCCR1A = bit (WGM10) | bit (WGM11) | bit (COM1B1); // fast PWM, clear OC1B on compare
  TCCR1B = bit (WGM12) | bit (WGM13) | bit (CS10);   // fast PWM, no prescaler
//  TCCR1B = bit (WGM10) | bit (WGM11) | bit (COM1B1);
  OCR1A =  timer1_OCR1A_Setting - 1;                 // zero relative  
//  OCR1B =  timer1_OCR1A_Setting - 1;                 // zero relative  

  // Serial.begin(9600);
  Serial.begin(19200);
  Serial.println("Start");
  
  // Temperature Setup
  sensors.begin();                    //Start Library
  sensors.requestTemperatures();      // Send the command to get temperatures
  sensor0_temp = sensors.getTempCByIndex(0); //Set Input to Current Temperature

  setPoint = 28;                      //Inintialize desired Temperature in Deg C

  double temp0Min = 25;
  double temp0Max = 50;
  double temp0Tgt = setPoint;

  fan_instance_autofill_tachs(fan0);

//  double temp1Min = 25;
//  double temp1Max = 50;
//  double temp1Tgt = setPoint;
//  double temp2Min = 25;
//  double temp2Max = 50;
//  double temp2Tgt = setPoint;
//  double temp3Min = 25;
//  double temp3Max = 50;
//  double temp3Tgt = setPoint;

  // PID Setup
  innerPID.SetMode(AUTOMATIC);
  innerPID.SetOutputLimits(innerS_min, innerS_max);

  outerPID.SetMode(AUTOMATIC);
  //TCCR2B = TCCR2B & 0b11111000 | 0x01;  //adjust the PWM Frequency, note: this changes timing like delay()
 
  loopCounter=0;

  // After requesting a temperature sample from a dallas ds18b20, we have to wait for the sensor to respond back
  sensors.setWaitForConversion(false);  // make it async
  sensors.requestTemperatures();
}


// input  = temp
// output = fan rpm value
// output = relay, OTP over temperature timeout period exceeded
void outer_loop()
{
  Serial.println("");
  Serial.println("outer_loop()");

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

  // Serial.print("  ");
  // Serial.print("pot0_val=");
  // Serial.print(pot0_val);

  Serial.print("    ");
  Serial.print("sensor0_temp=");
  Serial.print(sensor0_temp);


  Serial.print(",   ");
  Serial.print("pidOutput=");
  Serial.println(Output);

  Serial.println("");

}


// input  = current fan rpm
// output = desired fan rpm
// output = relay, fan tach failed
void inner_loop()
{
  Serial.println("  inner_loop()");

  clear_tachs();
  start = millis();       
  delay(innerS_delay);
  stop  = millis();
  ms_ellapsed = stop - start;

  fan0.last_tach = fan0.tach;

  Serial.print("     ");
  Serial.print("ms_ellapsed=");
  Serial.print(ms_ellapsed); 

  Serial.print(",    ");
  Serial.print("fan0.last_tach=");
  Serial.print(fan0.last_tach);

  Serial.print("     ");
  Serial.print("fan0.tach_target=");
  Serial.print(fan0.tach_target);

  fan0.rpm = (fan0.last_tach * 60 * 1000) / (2 * ms_ellapsed);
  // fan1_rpm = (fan1_last_tach * 60 * 1000) / ms_ellapsed;

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
  double gap = (fan0.tach_target - fan0.tach) / fan0.tach_target; //distance away from setpoint

  Serial.print("     ");
  Serial.print("gap=");
  Serial.print(gap);

  if(abs(gap) < tach_threshold_mid_fine)
  {
    // Close distance to setPoint
    innerPID.SetTunings(ip_fine.kp, ip_fine.ki, ip_fine.kd);
  }
  else if(abs(gap) < tach_threshold_near_mid)
  {
    // Medium distance to setPoint
    innerPID.SetTunings(ip_mid.kp, ip_mid.ki, ip_mid.kd);
  }
  else
  {
     //Far from setPoint, be aggresive
     innerPID.SetTunings(ip_fast.kp, ip_fast.ki, ip_fast.kd);
  } 

  innerPID.Compute();
  fan_instance_moderate_pwm(gap, fan0);

  Serial.print(",   ");
  Serial.print("fan0.pwm_last (PID)=");
  Serial.println(fan0.pwm_last);

  // alter Timer 1 duty cycle in accordance with pot reading
  OCR1B = (((long) (fan0.pwm_last + 1) * timer1_OCR1A_Setting) / 1024L) - 1;

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



