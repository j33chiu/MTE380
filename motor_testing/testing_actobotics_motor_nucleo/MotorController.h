// https://github.com/adafruit/Adafruit-Motor-Shield-library/blob/master/AFMotor.cpp
// https://github.com/adafruit/Adafruit-Motor-Shield-library/blob/master/AFMotor.h
// custom motor shield code for use of arduino motor controller with nucleo F401RE

#ifndef MOTOR_CONTROLLER
#define MOTOR_CONTROLLER

#include <inttypes.h>
//#include "variant_NUCLEO_F401RE.h"


// arduino motor  |  nucleo pin  |  default freq
// M1             |  D11 / PA7
// M2             |  D3 / PB3
// M3             |  D6 / PB10
// M4             |  D5 / PB4
#define M1_PWM_PIN      199 //PA7
#define M2_PWM_PIN      19//PB3
#define M3_PWM_PIN      26//PB10
#define M4_PWM_PIN      20//PB4

#define MOTOR_LATCH      198//PA6    // PA6, D12
#define MOTOR_CLK        21//PB5     // PB5, D4
#define MOTOR_ENABLE     8//PA8     // PA8, D7
#define MOTOR_DATA       9//PA9     // PA9, D8

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
  friend class DCMotor;
  void latch_tx(void);
};



// end of motor setup

#endif