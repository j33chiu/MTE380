// https://github.com/adafruit/Adafruit-Motor-Shield-library/blob/master/AFMotor.cpp
// https://github.com/adafruit/Adafruit-Motor-Shield-library/blob/master/AFMotor.h
// custom motor shield code for use of arduino motor controller with nucleo F401RE

#ifndef MOTOR_CONTROLLER
#define MOTOR_CONTROLLER

#include <inttypes.h>
#include "stm32f401xe.h"
#include "Arduino.h"

// arduino motor  |  nucleo pin  |  default freq
// M1             |  D11 / PA7
// M2             |  D3 / PB3
// M3             |  D6 / PB10
// M4             |  D5 / PB4
#define M1_PWM_PIN      PA7
#define M2_PWM_PIN      PB3
#define M3_PWM_PIN      PB10
#define M4_PWM_PIN      PB4

#define MOTOR_LATCH      12    // PA6
#define MOTOR_CLK        4     // PB5
#define MOTOR_ENABLE     7     // PA8
#define MOTOR_DATA       8     // PA9

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

static uint8_t latch_state; 

class MotorController {
public:
  MotorController(void);
  void enable(void);
  void latch_tx(void);
};



// end of motor setup

#endif