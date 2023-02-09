// TODO: Adjust aeverything if gyro-z is inversed (Ultrasonics, Motors)
#include <SPI.h>
#include <assert.h>
#include <Wire.h>

// ICM20948 IMU sensor
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "HardwareTimer.h"

// L9958 slave select pins for SPI
#define SS_M1 PA7
#define SS_M2 PA6
#define SS_M3 PA5
#define SS_M4 PA0

// L9958 DIRection pins
#define RIGHT_MOTOR_DIR PA10 // DIR_M1
#define DIR_M2 PB3
#define DIR_M3 PB5
#define LEFT_MOTOR_DIR PA8 // DIR_M4

// L9958 PWM pins
#define RIGHT_MOTOR PC7 // PWM_M1
#define PWM_M2 PB6    // Timer1
#define PWM_M3 PB4
#define LEFT_MOTOR PB10 // Timer0, PWM_M4

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS PA9

// i2c address
#define ICM20948_addr 0x69

// i2c lines: https://danieleff.github.io/STM32GENERIC/board_Nucleo_F401RE/
#define i2c_2_scl PB15
#define i2c_2_sda PB1

// Ultrasonic pins
#define left_US_echo PC3
#define left_US_trig PC2
#define right_US_echo PB13
#define right_US_trig PB14
#define front_US_echo PC10
#define front_US_trig PC12

// Button pin
#define buttonPin PC13

int start;

// Motor variables
int right_pwm, left_pwm;
int right_dir, left_dir;
int speed;
// int pwm1, pwm2, pwm3, pwm4;  
// int dir1, dir2, dir3, dir4;

// Gyro variables
TwoWire i2c_2(i2c_2_sda, i2c_2_scl);

// ICM20948 IMU
Adafruit_ICM20948 icm;

// gyro measuring variables
float poll_rate = 250.0f; // 200-250 Hz is doable, reduce to 200Hz if need additional processing per loop

float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;

float angle_turn = 0;
float angle_tilt = 0;
float angle_roll = 0;

long loop_time;
long last_measure;


// Returns the left ultrasonic readings in cm
long read_left_sensor() {
  digitalWrite(left_US_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(left_US_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(left_US_trig, LOW);
  long left_US_pulse_duration = pulseIn(left_US_echo, HIGH);
  return left_US_pulse_duration * 0.034 / 2;
}

// Returns the right ultrasonic readings in cm 
long read_right_sensor() {
  digitalWrite(right_US_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(right_US_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(right_US_trig, LOW);
  long right_US_pulse_duration = pulseIn(right_US_echo, HIGH);
  return right_US_pulse_duration * 0.034 / 2;
}

// Returns the front ultrasonic readings in cm 
long read_front_sensor() {
  digitalWrite(front_US_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(front_US_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(front_US_trig, LOW);
  long front_US_pulse_duration = pulseIn(front_US_echo, HIGH);
  return front_US_pulse_duration * 0.034 / 2;
}


// Function to accelerate robot given initial pwm, the final pwm we want, increment pwm interval and if robot is inverted
void accelerate(int initial_pwm, int max_pwm, int pwm_interval, int inverted) {
  if (inverted) {
    right_dir = !right_dir;
    left_dir = !left_dir;
  }

  for (int pwm = initial_pwm; pwm <= max_pwm; pwm += pwm_interval) {
    right_pwm = pwm;
    left_pwm = pwm;
    speed = pwm
    analogWrite(RIGHT_MOTOR, speed);
    digitalWrite(RIGHT_MOTOR_DIR, right_dir);
    analogWrite(LEFT_MOTOR, speed);
    digitalWrite(LEFT_MOTOR_DIR, left_dir);

    digitalWrite(ENABLE_MOTORS, LOW);
    delay(50);
  }
}

// Function to decelerate robot given initial pwm, the final pwm we want, increment pwm interval and if robot is inverted
void decelerate(int initial_pwm, int min_pwm, int pwm_interval, int inverted) {
  if (inverted) {
    right_dir = !right_dir;
    left_dir = !left_dir;
  }

  for (int pwm = initial_pwm; pwm >= min_pwm; pwm -= pwm_interval) {
    right_pwm = pwm;
    left_pwm = pwm;
    speed = pwm;
    analogWrite(RIGHT_MOTOR, right_pwm);
    digitalWrite(RIGHT_MOTOR_DIR, right_dir);
    analogWrite(LEFT_MOTOR, left_pwm);
    digitalWrite(LEFT_MOTOR_DIR, left_dir);

    digitalWrite(ENABLE_MOTORS, LOW);
    delay(50);
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  pinMode(buttonPin, INPUT);

  // Setup left ultrasonic sensor
  pinMode(left_US_trig, OUTPUT);
  pinMode(left_US_echo, INPUT);

  // Setup right ultrasonic sensor
  pinMode(right_US_trig, OUTPUT);
  pinMode(right_US_echo, INPUT);

  // Setup front ultrasonic sensor
  pinMode(front_US_trig, OUTPUT);
  pinMode(front_US_echo, INPUT);

  // TODO: Review Motor controller code and decide which motor slots to use
  // put your setup code here, to run once:
  unsigned int configWord;
  
    // put your setup code here, to run once:
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);  // HIGH = not selected
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);
  
  // L9958 DIRection pins
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  
  // L9958 PWM pins
  pinMode(RIGHT_MOTOR, OUTPUT);  digitalWrite(RIGHT_MOTOR, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(LEFT_MOTOR, OUTPUT);  digitalWrite(LEFT_MOTOR, LOW);    // Timer0
  
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled
  
  /******* Set up L9958 chips *********
  ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate
  '9 - ISR - current slew rate
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */
  
  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;
  
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high
  
  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH); 
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);
  
  // Reduce the PWM frequency to about 8kHz
  // Note, this will screw up the timer functions that use Timer0 such as millis()
  //setPwmFrequency(PWM_M1, 8);
  //setPwmFrequency(PWM_M3, 8);

  // setup ICM20948 IMU with i2c_2 sda and scl connections
  // i2c_2.begin();
  // if (!icm.begin_I2C(ICM20948_addr, &i2c_2)) {
  //   Serial.println("unable to setup ICM20948 IMU");
  //   while (1) { // pause
  //     delay(10);
  //   }
  // }
  // Serial.println("Successfully setup ICM20948 IMU");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  // icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  // icm.setAccelRateDivisor(4095);
  // icm.setGyroRateDivisor(255);
  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);

  // calibrate gyro
  // read from ICM20948 IMU
  // sensors_event_t accel;
  // sensors_event_t gyro;
  // sensors_event_t mag;
  // sensors_event_t temp;
  // for (int i = 0; i < 10000; ++i) {
  //   icm.getEvent(&accel, &gyro, &temp, &mag);
  //   gx_offset += gyro.gyro.x;
  //   gy_offset += gyro.gyro.y;
  //   gz_offset += gyro.gyro.z;
  //   delay(3);
  // }

  // gx_offset /= 10000.0f;
  // gy_offset /= 10000.0f;
  // gz_offset /= 10000.0f;

  // loop_time = micros();
  // last_measure = micros();
}

void loop() {
  // Check if button is pressed, press reset button to stop program
  if (digitalRead(buttonPin)) {} else {start = 1;} 

  // if button has been pressed, start program
  if (start) {

    // read from ICM20948 IMU
    // sensors_event_t gyro;
    // ((Adafruit_ICM20X_Gyro*)icm.getGyroSensor())->getEvent(&gyro);
    // long current_measure = micros();
    // float elaspsed = current_measure - last_measure;
    // last_measure = current_measure;

    // // Adjust gyro with offsets
    // gyro.gyro.x -= gx_offset;
    // gyro.gyro.y -= gy_offset;
    // gyro.gyro.z -= gz_offset;

    // float gx_rad = gyro.gyro.x * (elaspsed / 1000000.0f);
    // float gy_rad = gyro.gyro.y * (elaspsed / 1000000.0f);
    // float gz_rad = gyro.gyro.z * (elaspsed / 1000000.0f);

    // float gx_deg = gx_rad * 180.0f / PI;
    // float gy_deg = gy_rad * 180.0f / PI;
    // float gz_deg = gz_rad * 180.0f / PI;

    // // IMU must be flat to calibrate
    // angle_roll += gx_deg; //(gx_deg < 0.3f) ? 0.0f : gx_deg;
    // angle_tilt += gy_deg; //(gy_deg < 0.3f) ? 0.0f : gy_deg;
    // angle_turn += gz_deg; //(gz_deg < 0.3f) ? 0.0f : gz_deg;

    speed = 0;
    right_pwm = speed;
    left_pwm = speed;
    long front_US_dist = read_front_sensor();
    delay(20);
    long left_US_dist = read_left_sensor();
    delay(20);
    long right_US_dist = read_right_sensor();
    delay(20);

    // TODO: Decide factor to ramp up motors or start initally at max speed
    // TODO: Direction stuff
    accelerate(speed, 255, 10, 0);

    // if front sensor is approaching a wall, cut to around half speed
    if (front_US_dist < 25) {
      decelerate(speed, 120, 10, 0);
    }

    // if left sensor reading is past edge, reduce right motor speed 
    if (left_US_dist < 10) {

      // Make small adjustements
      // TODO: Test adjustement
      analogWrite(RIGHT_MOTOR, right_pwm - 5);
      digitalWrite(RIGHT_MOTOR_DIR, right_dir);
      digitalWrite(ENABLE_MOTORS, LOW);
      delay(50);

      analogWrite(RIGHT_MOTOR, right_pwm);
      digitalWrite(RIGHT_MOTOR_DIR, right_dir);
      digitalWrite(ENABLE_MOTORS, LOW);
    }

    // if right sensor reading is past edge, reduce left motor speed
    if (right_US_dist < 10) {
      // Make small adjustements
      // TODO: Test adjustement
      analogWrite(LEFT_MOTOR, left_pwm - 5);
      digitalWrite(LEFT_MOTOR_DIR, left_dir);
      digitalWrite(ENABLE_MOTORS, LOW);
      delay(50);

      analogWrite(LEFT_MOTOR, left_pwm);
      digitalWrite(LEFT_MOTOR_DIR, left_dir);
      digitalWrite(ENABLE_MOTORS, LOW);
    }

    Serial.print("Front edge distance: ");
    Serial.print(front_US_dist);
    Serial.println("cm. ");
    Serial.print("Left edge distance: ");
    Serial.print(left_US_dist);
    Serial.println("cm. ");
    Serial.print("Right edge distance: ");
    Serial.print(right_US_dist);
    Serial.println("cm. ");
    // Serial.print("Roll angle: ");
    // Serial.print(angle_roll);
    // Serial.println(" degrees (x)");
    // Serial.print("Tilt angle: ");
    // Serial.print(angle_tilt);
    // Serial.println(" degrees (y)");
    // Serial.print("Turn angle: ");
    // Serial.print(angle_turn);
    // Serial.println(" degrees (z)");

    delay(20);
  }
}
