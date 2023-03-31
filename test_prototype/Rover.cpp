#include "Rover.h"

#include "DCMotor.h"

namespace Rover {

// motor objects for left and right tracks
DCMotor right(1);
DCMotor left(3);

// sensor variables
const int FRONT_US_TRIG = PC12;
const int FRONT_US_ECHO = PC10;
const int LEFT_US_TRIG = PC2;
const int LEFT_US_ECHO = PC3;
const int RIGHT_US_TRIG = PB14;
const int RIGHT_US_ECHO = PB13;

const int ICM20948_ADDR = 0x69;
const int I2C_SCL = PB8; //PB15;
const int I2C_SDA = PB9; //PB1;
TwoWire i2c(I2C_SDA, I2C_SCL);

// TOF laser sensor
const int SEN0245_ADDR = 0x50;
DFRobot_VL53L0X tof_sensor;

// gyro variables
Adafruit_ICM20948 icm;
float poll_rate = 250.0f; // 200-250 Hz is doable, reduce to 200Hz if need additional processing per loop

float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

float angle_turn = 0;
float angle_tilt = 0;
float angle_roll = 0;

long loop_time;
long last_measure;

// rover variables
bool ready = false;                       // rover has done setup properly
uint8_t currentPercentSpeed = 0;          // current rover speed in percent (in any direction, including turning speed)

RotateType rotateType = RotateType::GYRO;
MoveDirection currentMoveDirection = MoveDirection::STOPPED;


void init() {

  // ultrasonic sensor setup:
  // pinMode(FRONT_US_TRIG, OUTPUT);
  // pinMode(FRONT_US_ECHO, INPUT);
  // pinMode(LEFT_US_TRIG, OUTPUT);
  // pinMode(LEFT_US_ECHO, INPUT);
  // pinMode(RIGHT_US_TRIG, OUTPUT);
  // pinMode(RIGHT_US_ECHO, INPUT);
  

  // motor setup
  DCMotor::setupMotors();
  left.init_PWM();
  right.init_PWM();
  DCMotor::enableMotors();

  // gyro setup
  i2c.begin();
  if (!icm.begin_I2C(ICM20948_ADDR, &i2c)) {
    Serial.println("unable to setup ICM20948 IMU");
  } else {
    initGyro();
  }

  // setup TOF sensor
  Wire.begin(); // PB3 (SDA), PB10 (SCL)
  tof_sensor.begin(SEN0245_ADDR); // https://github.com/DFRobot/I2C_Addresses
  tof_sensor.setMode(
    DFRobot_VL53L0X::eModeState::eContinuous, 
    DFRobot_VL53L0X::ePrecisionState::eHigh);
  tof_sensor.start();

  ready = true;
}

void initGyro() {
  Serial.println("Successfully setup ICM20948 IMU");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  icm.setAccelRateDivisor(4095);
  icm.setGyroRateDivisor(255);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);

  // calibrate gyro
  // read from ICM20948 IMU
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  for (int i = 0; i < 1000; ++i) {
    icm.getEvent(&accel, &gyro, &temp, &mag);
    gx_offset += gyro.gyro.x;
    gy_offset += gyro.gyro.y;
    gz_offset += gyro.gyro.z;
    delay(3);
  }
  gx_offset /= 1000.0f;
  gy_offset /= 1000.0f;
  gz_offset /= 1000.0f;
  loop_time = micros();
  last_measure = micros();
}

void forwards(uint8_t percentSpeed) {
  if (!ready) return;
  if (percentSpeed < 0 || percentSpeed > 100) return;
  if (currentMoveDirection == MoveDirection::BACKWARDS || currentMoveDirection == MoveDirection::TURNING) stop();

  left.run(MotorState::FORWARD);
  right.run(MotorState::BACKWARD);
  currentMoveDirection = MoveDirection::FORWARDS;

  uint8_t interval = percentSpeed - currentPercentSpeed;

  for (int speed = currentPercentSpeed; speed != percentSpeed; speed += (interval < 0 ? -1 : 1)) {
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    currentPercentSpeed = speed;
    delay(10);
  }
}

void backwards(uint8_t percentSpeed) {
  if (!ready) return;
  if (percentSpeed < 0 || percentSpeed > 100) return;
  if (currentMoveDirection == MoveDirection::FORWARDS || currentMoveDirection == MoveDirection::TURNING) stop();

  left.run(MotorState::BACKWARD);
  right.run(MotorState::FORWARD);
  currentMoveDirection = MoveDirection::BACKWARDS;

  uint8_t interval = percentSpeed - currentPercentSpeed;

  for (int speed = currentPercentSpeed; speed != percentSpeed; speed += (interval < 0 ? -1 : 1)) {
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    currentPercentSpeed = speed;
    
    tickGyro();
    tickGyro();
    tickGyro();
  }
}

void stop() {
  if (!ready) return;

  uint8_t leftSpeed = left.getMotorSpeedPercent();
  uint8_t rightSpeed = right.getMotorSpeedPercent();
  bool leftMoving = (leftSpeed > 0);
  bool rightMoving = (rightSpeed > 0);

  while (leftMoving || rightMoving) {
    if (leftMoving) left.setSpeedPercent(leftSpeed - 1);
    if (rightMoving) right.setSpeedPercent(rightSpeed - 1);
    leftSpeed = left.getMotorSpeedPercent();
    rightSpeed = right.getMotorSpeedPercent();
    leftMoving = (leftSpeed > 0);
    rightMoving = (rightSpeed > 0);
    
    tickGyro();
    tickGyro();
    tickGyro();
  }

  currentPercentSpeed = 0;
  currentMoveDirection = MoveDirection::STOPPED;
}

void leftRotate(uint8_t percentSpeed) {
  if (!ready) return;
  if (percentSpeed < 0 || percentSpeed > 100) return;

  stop(); // rover should stop before rotating

  left.run(MotorState::BACKWARD);
  right.run(MotorState::BACKWARD);

  uint8_t interval = percentSpeed - currentPercentSpeed;

  for (int speed = currentPercentSpeed; speed != percentSpeed; speed += (interval < 0 ? -1 : 1)) {
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    currentPercentSpeed = speed;
    //delay(10);
    // might need to tick gyro here
    tickGyro();
    tickGyro();
    tickGyro();
    // ~9m delay from ticking gyro 3x
  }
}

void rightRotate(uint8_t percentSpeed) {
  if (!ready) return;
  if (percentSpeed < 0 || percentSpeed > 100) return;

  stop(); // rover should stop before rotating

  left.run(MotorState::FORWARD);
  right.run(MotorState::FORWARD);

  uint8_t interval = percentSpeed - currentPercentSpeed;

  for (int speed = currentPercentSpeed; speed != percentSpeed; speed += (interval < 0 ? -1 : 1)) {
    left.setSpeedPercent(speed);
    right.setSpeedPercent(speed);
    currentPercentSpeed = speed;
    // delay(10);
    // might need to tick gyro here
    tickGyro();
    tickGyro();
    tickGyro();
    // ~9m delay from ticking gyro 3x
  }
}


// sensor information
long distanceFront() {
  if (!ready) return -1;

  digitalWrite(FRONT_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_US_TRIG, LOW);
  long pulseDuration = pulseIn(FRONT_US_ECHO, HIGH);
  return pulseDuration * 0.034 / 2;
}

long distanceFrontLaser() {
  if (!ready) return -1;
  return tof_sensor.getDistance();
}

bool atDestination() {
  // TODO: use colour sensor
  return false;
}

void tickGyro() {
  // read from ICM20948 IMU
  sensors_event_t gyro;
  ((Adafruit_ICM20X_Gyro*)icm.getGyroSensor())->getEvent(&gyro);
  long current_measure = micros();
  float elaspsed = current_measure - last_measure;
  last_measure = current_measure;

  // Adjust gyro with offsets
  gyro.gyro.x -= gx_offset;
  gyro.gyro.y -= gy_offset;
  gyro.gyro.z -= gz_offset;

  float gx_rad = gyro.gyro.x * (elaspsed / 1000000.0f);
  float gy_rad = gyro.gyro.y * (elaspsed / 1000000.0f);
  float gz_rad = gyro.gyro.z * (elaspsed / 1000000.0f);

  float gx_deg = gx_rad * 180.0f / PI;
  float gy_deg = gy_rad * 180.0f / PI;
  float gz_deg = gz_rad * 180.0f / PI;

  // IMU must be flat to calibrate
  angle_roll += gx_deg; //(gx_deg < 0.3f) ? 0.0f : gx_deg;
  angle_tilt += gy_deg; //(gy_deg < 0.3f) ? 0.0f : gy_deg;
  angle_turn += gz_deg; //(gz_deg < 0.3f) ? 0.0f : gz_deg;
}

float turnAngle() {
  if (!ready) return -1;
  return angle_turn;
}

float tiltAngle() {
  if (!ready) return NULL;
  return angle_tilt;
}

void resetTurn() {
  angle_turn = 0;
}

void resetTilt() {
  angle_tilt = 0;
}

bool isVertical() {
  if (!ready) return false;
  // calibrate this 70 degree value
  if (angle_tilt > 60 || angle_tilt < -60) return true;
  return false;
}

bool isUpsideDown() {
  if (!ready) return false;
  // if tilt_angle is around +/- 180 return true
  // or use magnetometer
}

bool shouldEStop() {
  if (!ready) return false;
}
  
}