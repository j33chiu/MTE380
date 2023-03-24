#include <Wire.h>

// ICM20948 IMU sensor
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// i2c addresses
const int ICM20948_addr = 0x69;

const int board_green_led = PA5;

// i2c lines: https://danieleff.github.io/STM32GENERIC/board_Nucleo_F401RE/
const int i2c_2_scl = PB8;
const int i2c_2_sda = PB9;

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


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause

  // setup nucleo onboard green led
  pinMode(board_green_led, OUTPUT);

  // setup ICM20948 IMU with i2c_2 sda and scl connections
  i2c_2.begin();
  if (!icm.begin_I2C(ICM20948_addr, &i2c_2)) {
    Serial.println("unable to setup ICM20948 IMU");
    while (1) { // pause
      delay(10);
    }
  }
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

void loop() {
  //long startTime = micros();


  // read from ICM20948 IMU
  //sensors_event_t accel;
  sensors_event_t gyro;
  //sensors_event_t mag;
  //sensors_event_t temp;
  ((Adafruit_ICM20X_Gyro*)icm.getGyroSensor())->getEvent(&gyro);
  long current_measure = micros();
  float elaspsed = current_measure - last_measure;
  last_measure = current_measure;
  //icm.getEvent(&accel, &gyro, &temp, &mag);

  // adjust gyro with offsets
  gyro.gyro.x -= gx_offset;
  gyro.gyro.y -= gy_offset;
  gyro.gyro.z -= gz_offset;

  //float gx_rad = gyro.gyro.x / poll_rate;
  //float gy_rad = gyro.gyro.y / poll_rate;
  //float gz_rad = gyro.gyro.z / poll_rate;
  float gx_rad = gyro.gyro.x * (elaspsed / 1000000.0f);
  float gy_rad = gyro.gyro.y * (elaspsed / 1000000.0f);
  float gz_rad = gyro.gyro.z * (elaspsed / 1000000.0f);

  float gx_deg = gx_rad * 180.0f / PI;
  float gy_deg = gy_rad * 180.0f / PI;
  float gz_deg = gz_rad * 180.0f / PI;

  // if the imu is not flat on the ground when calibrated, errors may propagate as time passes and robot moves
  angle_roll += gx_deg; //(gx_deg < 0.3f) ? 0.0f : gx_deg;
  angle_tilt += gy_deg; //(gy_deg < 0.3f) ? 0.0f : gy_deg;
  angle_turn += gz_deg; //(gz_deg < 0.3f) ? 0.0f : gz_deg;

  // logging:
  /*Serial.print("Mag(uT): (");
  Serial.print(mag.magnetic.x);
  Serial.print(", ");
  Serial.print(mag.magnetic.y);
  Serial.print(", ");
  Serial.print(mag.magnetic.z);
  Serial.print(")");
  Serial.print(" \tGyro(rad/s): (");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.print(")");
  Serial.print(" \tpitch: ");
  Serial.print(gyro.gyro.pitch);
  Serial.print(" \troll: ");
  Serial.print(gyro.gyro.roll);
  Serial.print("(");
*/

  Serial.print("turn angle: ");
  Serial.print(angle_turn);
  Serial.print(" \ttilt angle: ");
  Serial.println(angle_tilt);

  //while(micros() - loop_time < 1000000.0f / poll_rate); // waits until the loop has taken 4000 us (4ms), 1000000.0f us in 1s
  loop_time = micros();
  
  /*long endTime = micros();
  Serial.print(endTime - startTime);
  Serial.println("us"); */

  //delay(1000);
}
