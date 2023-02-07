#include <SPI.h>
#include <assert.h>

#include "HardwareTimer.h"

// L9958 slave select pins for SPI
#define SS_M1 PA7
#define SS_M2 PA6
#define SS_M3 PA5
#define SS_M4 PA0

// L9958 DIRection pins
#define DIR_M1 PA10
#define DIR_M2 PB3
#define DIR_M3 PB5
#define DIR_M4 PA8

// L9958 PWM pins
#define PWM_M1 PC7
#define PWM_M2 PB6    // Timer1
#define PWM_M3 PB4
#define PWM_M4 PB10     // Timer0

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS PA9

int pwm1, pwm2, pwm3, pwm4;
int dir1, dir2, dir3, dir4;

void setup() {
  // put your setup code here, to run once:
  unsigned int configWord;
  
    // put your setup code here, to run once:
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);  // HIGH = not selected
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);
  
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0
  
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled
  
  /******* Set up L9958 chips *********
  ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate
  '9 - ISR - current slew rate
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */
  
  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;
  
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high
  
  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH); 
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);
  
  // Reduce the PWM frequency to about 8kHz
  // Note, this will screw up the timer functions that use Timer0 such as millis()
  //setPwmFrequency(PWM_M1, 8);
  //setPwmFrequency(PWM_M3, 8);
}

void loop() {
  // put your main code here, to run repeatedly:
  dir1 = !dir1;
  
  // ramp speed up for all motors
  for (pwm1 = 0; pwm1 < 256; pwm1 +=5) {
    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M4, pwm1);  digitalWrite(DIR_M4, dir1);
    
    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    delay(50);
  }
  delay(2000); // delay 2 sec
  
  // ramp speed down for all motors
  for (pwm1 = 255; pwm1 >= 0; pwm1 -=5) {
    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
    analogWrite(PWM_M4, pwm1);  digitalWrite(DIR_M4, dir1);
    
    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    delay(50);
  } 
  delay(2000);

}
