#include <assert.h>

#include "DCMotor.h"

DCMotor m1(1);
DCMotor m3(3);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause
  Serial.println("Motor test!");

  DCMotor::setupMotors();
  m1.init_PWM();
  m3.init_PWM();

  DCMotor::enableMotors();
}

void loop() {
  Serial.println("loop start");
  // ramp speed up for all motors
  m1.run(MotorState::FORWARD);
  m3.run(MotorState::FORWARD);
  for (int i = 0; i <= 100; i += 5) {
    m1.setSpeedPercent(i);
    m3.setSpeedPercent(i);
    delay(50);
  }
  delay(10000);
  for (int i = 100; i >= 0; i -= 5) {
    m1.setSpeedPercent(i);
    m3.setSpeedPercent(i);
    delay(50);
  }
  delay(1000);
  
  m1.run(MotorState::BACKWARD);
  m3.run(MotorState::BACKWARD);
  for (int i = 0; i <= 100; i += 5) {
    m1.setSpeedPercent(i);
    m3.setSpeedPercent(i);
    delay(50);
  }
  delay(10000);
  for (int i = 100; i >= 0; i -= 5) {
    m1.setSpeedPercent(i);
    m3.setSpeedPercent(i);
    delay(50);
  }
  delay(1000);

}
