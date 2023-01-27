#include <Wire.h>

// pinout: https://github.com/RobTillaart/GY521

// i2c addresses
const int GY521_addr = 0x69; // AD0 pin connected to vcc, 0x68 if connected to ground

const int board_green_led = PA5;

// i2c lines: https://danieleff.github.io/STM32GENERIC/board_Nucleo_F401RE/
const int i2c_2_scl = PB10;
const int i2c_2_sda = PB3;

TwoWire i2c_2(i2c_2_sda, i2c_2_scl);

// angle measurments
int gx, gy, gz;
long gx_offset, gy_offset, gz_offset;
bool set_gyro_angles;

long accel_x, accel_y, accel_z, accel_total_vector;
float angle_roll_accel, angle_pitch_accel;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_out, angle_roll_out;

long loop_time; // in microseconds
int temp;

// benchmark
int lastMillis = 0;

void read_gyro() {

}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause

  // setup nucleo onboard green led
  pinMode(board_green_led, OUTPUT);

  // setup GY521 with i2c_2 sda and scl connections
  i2c_2.begin();
  if (!icm.begin_I2C(GY521_addr, &i2c_2)) {
    Serial.println("unable to setup GY521 IMU");
    while (1) { // pause
      delay(10);
    }
  }
  Serial.println("Successfully setup ICM20948 IMU");
  
  // calibrate gyro
  for (int i = 0; i < 1000; ++i) {
    read_gyro();
    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    delay(5);
  } // results in ~5s setup
  gx_offset /= 1000;
  gy_offset /= 1000;
  gz_offset /= 1000;

  loop_time = micros();
}

void loop() {
  read_gyro();

  // subtract offsets
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // calculate angles
  
  
  //delay(1000);
}

