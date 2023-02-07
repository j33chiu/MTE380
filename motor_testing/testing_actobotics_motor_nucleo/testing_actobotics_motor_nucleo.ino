#include "MotorController.h"
#include "DCMotor.h"
#include "stm32f401xe.h"
#include "Arduino.h"
#include "HardwareTimer.h"


//DCMotor m1(1);
DCMotor m2(2);
//DCMotor m3(3);
//DCMotor m4(4);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause
  Serial.println("Motor test!");
  
  // setup stm board for motors

/*
  TIM_TypeDef *Instance_led = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PA5), PinMap_PWM);
  uint32_t channel_led = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PA5), PinMap_PWM));

  // PB3: value = 19
  TIM_TypeDef *Instance_m2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(19), PinMap_PWM);
  uint32_t channel_m2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(19), PinMap_PWM));

  HardwareTimer *pwmTimerLed = new HardwareTimer(Instance_led);
  pwmTimerLed->setPWM(channel_led, PA5, 1000, 5);

  HardwareTimer *pwmTimerM2 = new HardwareTimer(Instance_m2);
  pwmTimerM2->setPWM(channel_m2, 19, 1000, 5);
*/

  Serial.print("PA6:");
  Serial.println(PA6);
  Serial.print("PB5:");
  Serial.println(PB5);
  Serial.print("PA8:");
  Serial.println(PA8);
  Serial.print("PA9:");
  Serial.println(PA9);

  pinMode(MOTOR_LATCH, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DATA, OUTPUT);
  pinMode(MOTOR_CLK, OUTPUT);

  m2.init_PWM();
  /*m1.init_PWM();
  m2.init_PWM();
  m3.init_PWM();
  m4.init_PWM();
*/
}

void loop() {


  Serial.println("loop start");



  m2.run(MotorState::FORWARD);
  //m2.run(FORWARD);
  m2.setSpeed(5);
  //analogWrite(PB3, 255);
  delay (2000);
  /*for (int i = 0; i < 100; i++) {
    m2.setSpeed(i);
    delay(10);
  }

  for (int i = 100; i != 0; i--) {
    m2.setSpeed(i);
    delay(10);
  }

  delay(500);

  Serial.println("backwards");
  //m2.run(MotorState::BACKWARD);
  m2.run(BACKWARD);
  for (int i = 0; i < 100; i++) {
    m2.setSpeed(i);
    delay(10);
  }

  for (int i = 100; i != 0; i--) {
    m2.setSpeed(i);
    delay(10);
  }*/
  m2.run(MotorState::RELEASE);
  //m2.run(RELEASE);

  delay(1000);
}
