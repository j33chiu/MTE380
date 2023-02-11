#ifndef DC_MOTOR
#define DC_MOTOR

#include <inttypes.h>
#include "stm32f401xe.h"
#include "Arduino.h"

#include "HardwareTimer.h"

// L9958 slave select pins for SPI
#define SS_M1 PA7
#define SS_M2 PA6
#define SS_M3 PA5
#define SS_M4 PA0

#define M1_PWM_PIN      PC7
#define M2_PWM_PIN      PB6
#define M3_PWM_PIN      PB4
#define M4_PWM_PIN      PB10

#define M1_DIR_PIN      PA10
#define M2_DIR_PIN      PB3
#define M3_DIR_PIN      PB5
#define M4_DIR_PIN      PA8

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS PA9

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
  void setSpeedPercent(uint8_t speedPercent);

  MotorState getMotorState(void);
  uint8_t getMotorSpeedPercent(void);

  static void enableMotors(void);
  static void disableMotors(void);
  static void setupMotors(void);

private:
  uint8_t motorNum = 0;
  int pwmPin;
  int dirPin;

  MotorState motorState = MotorState::RELEASE;
  uint8_t speedPercent = 0;

  // pwm variables
  TIM_TypeDef* htim = nullptr; 
  uint32_t htimChannel = 0;
  HardwareTimer* pwmTimer = nullptr;
};

#endif