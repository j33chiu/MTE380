#ifndef DC_MOTOR
#define DC_MOTOR

#include <inttypes.h>
#include "stm32f401xe.h"
#include "Arduino.h"

#include "HardwareTimer.h"
#include "MotorController.h"

enum class MotorState : uint8_t {
  FORWARD,
  BACKWARD,
  BRAKE,
  RELEASE
};

class DCMotor {
public:
  DCMotor(uint8_t motorNum);
  void init_PWM(void);
  void run(MotorState motorState);
  void setSpeed(uint8_t speedPercent);

private:
  static MotorController MC;
  uint8_t motorNum;

  // pwm variables
  TIM_TypeDef* htim = nullptr; 
  uint32_t htimChannel = 0;
  HardwareTimer* pwmTimer = nullptr;
};

#endif