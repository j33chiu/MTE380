#include "DCMotor.h"

// declare static motorcontroller for DCMotor class:
MotorController DCMotor::MC;

DCMotor::DCMotor(uint8_t motorNum) {
  this->motorNum = motorNum;
  
  MC.enable();

  switch(motorNum) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  }
}

void DCMotor::init_PWM(void) {

  uint32_t pin;
  switch(motorNum) {
  case 1:
    pin = M1_PWM_PIN; break;
  case 2:
    pin = M2_PWM_PIN; break;
  case 3:
    pin = M3_PWM_PIN; break;
  case 4:
    pin = M4_PWM_PIN; break;
  }

  this->htim = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  this->htimChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  this->pwmTimer = new HardwareTimer(this->htim);
  this->pwmTimer->setPWM(this->htimChannel, pin, 8000, 0); // 0% duty cycle to start with (motors standstill)

}

void DCMotor::run(MotorState motorState) {
  uint8_t a, b;
  switch (motorNum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (motorState) {
  case MotorState::FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case MotorState::BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case MotorState::RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void DCMotor::setSpeed(uint8_t speedPercent) {
  switch (motorNum) {
  case 1:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M1_PWM_PIN, 8000, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 2:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M2_PWM_PIN, 8000, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 3:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M3_PWM_PIN, 8000, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  case 4:
    this->pwmTimer->pause();
    this->pwmTimer->setPWM(this->htimChannel, M4_PWM_PIN, 8000, speedPercent);
    this->pwmTimer->refresh();
    this->pwmTimer->resume();
    break;
  }

  if(Serial) Serial.println(latch_state);
}