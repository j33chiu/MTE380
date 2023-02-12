#include "HardwareTimer.h"
#include "DCMotor.h"

// Motor variables
DCMotor right(1);
DCMotor left(3);
int current_speed;


// Function to accelerate robot given initial pwm, the final pwm we want, increment pwm interval and if robot is inverted
void accelerate(int final_speed, int interval, int backward) {

  for (int speed = current_speed; speed <= final_speed; speed += interval) {
    if (backward) {
      left.run(MotorState::BACKWARD);
      right.run(MotorState::FORWARD);
    } else {
      left.run(MotorState::FORWARD);
      right.run(MotorState::BACKWARD);
    }   
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    current_speed = speed;
    delay(10);
  }
}

// Function to decelerate robot given initial pwm, the final pwm we want, increment pwm interval and if robot is inverted
void decelerate(int final_speed, int interval, int backward) {
  for (int speed = current_speed; speed >= final_speed; speed -= interval) {
    if (backward) {
      left.run(MotorState::BACKWARD);
      right.run(MotorState::FORWARD);
    } else {
      left.run(MotorState::FORWARD);
      right.run(MotorState::BACKWARD);
    }   
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    current_speed = speed;
    delay(10);
  }
}

// TODO: Set speed to 0 or enter motor state RELEASE
void stop() {
  left.setSpeedPercent(0);
  right.setSpeedPercent(0);
  current_speed = 0;
  // left.run(MotorState::RELEASE);
  // right.run(motorState::RELEASE);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // Setup Motor Controller
  DCMotor::setupMotors();
  left.init_PWM();
  right.init_PWM();

  DCMotor::enableMotors();

  current_speed = 0;
}

void loop() {
  Serial.println("Program started");
    // accelerate(100, 10, 0);
    // delay(2000);
    // decelerate(0, 10, 0);
    // delay(2000);
    // accelerate(100, 10, 1);
    // delay(2000);
    // decelerate(0, 10, 1);

    left.run(MotorState::FORWARD);
    left.setSpeedPercent(100);
    right.run(MotorState::FORWARD);
    right.setSpeedPercent(100);
    delay(2000);

    left.setSpeedPercent(0);
    right.setSpeedPercent(0);
    delay(2000);

    left.run(MotorState::BACKWARD);
    left.setSpeedPercent(100);
    right.run(MotorState::BACKWARD);
    right.setSpeedPercent(100);
    delay(2000);

    left.setSpeedPercent(0);
    right.setSpeedPercent(0);
    delay(2000);

    delay(2000);
}
