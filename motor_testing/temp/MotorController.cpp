#include "MotorController.h"

MotorController::MotorController(void) {

}

void MotorController::enable(void) {
  // setup latch
  pinMode(MOTOR_LATCH, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DATA, OUTPUT);
  pinMode(MOTOR_CLK, OUTPUT);

  latch_state = 0;
  latch_tx();  // "reset"

  digitalWrite(MOTOR_ENABLE, LOW);
}

void MotorController::latch_tx(void) {

  digitalWrite(MOTOR_LATCH, LOW);

  digitalWrite(MOTOR_DATA, LOW);

  for (int i = 0; i < 8; i++) {
    digitalWrite(MOTOR_CLK, LOW);

    if (latch_state & _BV(7 - i)) {
      digitalWrite(MOTOR_DATA, HIGH);
    } else {
      digitalWrite(MOTOR_DATA, LOW);
    }
    digitalWrite(MOTOR_CLK, HIGH);
  }
  digitalWrite(MOTOR_LATCH, HIGH);
}
