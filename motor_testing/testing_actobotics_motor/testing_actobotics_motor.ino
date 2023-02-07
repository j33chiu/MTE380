//#include "MotorController.h"
//#include "DCMotor.h"
//#include "stm32f401xe.h"
#include "Arduino.h"
//#include "HardwareTimer.h"



#include <AFMotor.h>

/*DCMotor m1(1);
DCMotor m2(2);
DCMotor m3(3);
DCMotor m4(4);*/
AF_DCMotor m2(2, MOTOR12_64KHZ);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause
  Serial.println("Motor test!");
  
  // setup stm board for motors

/*
  TIM_TypeDef *Instance_led = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PA5), PinMap_PWM);
  uint32_t channel_led = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PA5), PinMap_PWM));

  TIM_TypeDef *Instance_m2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PB3), PinMap_PWM);
  uint32_t channel_m2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PB3), PinMap_PWM));

  HardwareTimer *pwmTimerLed = new HardwareTimer(Instance_led);
  pwmTimerLed->setPWM(channel_led, PA5, 1000, 5);

  HardwareTimer *pwmTimerM2 = new HardwareTimer(Instance_m2);
  pwmTimerM2->setPWM(channel_m2, PB3, 1000, 5);*/

  /*m1.init_PWM();
  m2.init_PWM();
  m3.init_PWM();
  m4.init_PWM();
*/
}

void loop() {


  Serial.println("loop start");



  Serial.println("forwards");
  //m2.run(MotorState::FORWARD);
  m2.run(FORWARD);
  m2.setSpeed(5);
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

  Serial.println("releasing");
  //m2.run(MotorState::RELEASE);
  m2.run(RELEASE);

  delay(1000);
}
