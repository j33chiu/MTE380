#include "DCMotor.h"

#include <SPI.h>

DCMotor::DCMotor(uint8_t motorNum) {
  this->motorNum = motorNum;
  switch(motorNum) {
  case 1:
    this->pwmPin = M1_PWM_PIN; 
    this->dirPin = M1_DIR_PIN; 
    break;
  case 2:
    this->pwmPin = M2_PWM_PIN; 
    this->dirPin = M2_DIR_PIN; 
    break;
  case 3:
    this->pwmPin = M3_PWM_PIN; 
    this->dirPin = M3_DIR_PIN; 
    break;
  case 4:
    this->pwmPin = M4_PWM_PIN; 
    this->dirPin = M4_DIR_PIN; 
    break;
  }

}

void DCMotor::init_PWM(void) {

  this->htim = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmPin), PinMap_PWM);
  this->htimChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmPin), PinMap_PWM));

  this->pwmTimer = new HardwareTimer(this->htim);
  this->pwmTimer->setPWM(this->htimChannel, pwmPin, PWM_FREQ_CHOSEN, 0); // 0% duty cycle to start with (motors standstill)

}

void DCMotor::run(MotorState motorState) {
  this->motorState = motorState;
  switch (motorState) {
  case MotorState::FORWARD:
    Serial.println("forwards");
    digitalWrite(dirPin, LOW);
    break;
  case MotorState::BACKWARD:
    Serial.println("backwards");
    digitalWrite(dirPin, HIGH);
    break;
  case MotorState::RELEASE:
    Serial.println("release");
    //digitalWrite(M1_DIR_PIN, dir1);
    break;
  }
}

void DCMotor::setSpeedPercent(uint8_t speedPercent) {
  this->speedPercent = speedPercent;
  switch (motorNum) {
  case 1:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M1_PWM_PIN, PWM_FREQ_CHOSEN, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 2:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M2_PWM_PIN, PWM_FREQ_CHOSEN, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 3:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M3_PWM_PIN, PWM_FREQ_CHOSEN, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 4:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M4_PWM_PIN, PWM_FREQ_CHOSEN, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  }
}

void DCMotor::enableMotors(void) {
  digitalWrite(ENABLE_MOTORS, LOW);   // HIGH = disabled
}

void DCMotor::disableMotors(void) {
  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled
}

void DCMotor::setupMotors(void) {
  unsigned int configWord;

  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);  // HIGH = not selected
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);
  
  // L9958 DIRection pins
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M3_DIR_PIN, OUTPUT);
  
  // L9958 PWM pins
  pinMode(M1_PWM_PIN, OUTPUT);  digitalWrite(M1_PWM_PIN, LOW);
  //pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(M3_PWM_PIN, OUTPUT);  digitalWrite(M3_PWM_PIN, LOW);
  //pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0
  
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
  //digitalWrite(SS_M2, LOW);
  //SPI.transfer(lowByte(configWord));
  //SPI.transfer(highByte(configWord));
  //digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  //digitalWrite(SS_M4, LOW);
  //SPI.transfer(lowByte(configWord));
  //SPI.transfer(highByte(configWord));
  //digitalWrite(SS_M4, HIGH);
  
}