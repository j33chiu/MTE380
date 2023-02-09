#include <Wire.h>

// TOF laser sensor SEN0245
#include <DFRobot_VL53L0X.h>

// i2c addresses
const int SEN0245_addr = 0x50;

//const int board_green_led = PA5;

// i2c lines: https://danieleff.github.io/STM32GENERIC/board_Nucleo_F401RE/
const int i2c_1_scl = 19;//PB8;
const int i2c_1_sda = 18;//PB9;

// TOF laser sensor
DFRobot_VL53L0X tof_sensor;

// benchmark
int lastMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause

  // i2c connections
  //Wire.begin(i2c_1_sda, i2c_1_scl); // TOF laser sensor uses default Wire, so redefine pins
  Wire.begin();

  // setup nucleo onboard green led
  //pinMode(board_green_led, OUTPUT);

  // setup TOF laser sensor (i2c must be initialized first)
  tof_sensor.begin(SEN0245_addr); // https://github.com/DFRobot/I2C_Addresses
  tof_sensor.setMode(
    DFRobot_VL53L0X::eModeState::eContinuous, 
    DFRobot_VL53L0X::ePrecisionState::eHigh);
  tof_sensor.start();

  lastMillis = millis();
}

void loop() {


  // read from tof laser sensor
  Serial.print("TOF Sensor Distance: ");
  Serial.print(tof_sensor.getDistance());
  Serial.print("mm. (");

  int curTime = millis();
  Serial.print(curTime - lastMillis);
  Serial.println("ms)"); // 
  lastMillis = curTime;

  //delay(1000);
}
