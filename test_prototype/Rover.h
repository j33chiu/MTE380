#ifndef ROVER
#define ROVER

#include <Wire.h>

// ICM20948 IMU sensor
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

namespace Rover {

enum class RotateType : uint8_t {
  GYRO,           // gyro degrees 
  MAGNETOMETER,   // gyro magnetometer
  TIME,           // rotate for x amount of time
  ENCODER         // rotate based on encoders
};

enum class TurnDirection : uint8_t {
  RIGHT,
  LEFT
};

enum class MoveDirection : uint8_t {
  FORWARDS,
  BACKWARDS,
  STOPPED,
  TURNING
};

// setup
void init();

// movement functions
void forwards(uint8_t percentSpeed);                                      // move straight forwards
void backwards(uint8_t percentSpeed);                                     // move straight backwards
void stop();                                                              // full stop movement
void leftRotate(uint8_t percentSpeed = 100);                              // pivot left at specified speed percent
void rightRotate(uint8_t percentSpeed = 100);                             // pivot right at specified speed percent

// sensor information
long distanceFront();                                                     // get distance (in cm) to an object in front of the rover
bool atDestination();                                                     // get whether the target has been located
void tickGyro();                                                          // function to be called as frequently as possible to update from gyro
float turnAngle();                                                        // get current turn angle of the rover (off original starting position)
bool isVertical();                                                        // get whether or not the rover is vertical (on the wall)
bool isUpsideDown();                                                      // determines whether or not the rover is currently upsidedown
bool shouldEStop();                                                       // determines if something has gone wrong and rover needs to turn off to protect hardware                                           


}

#endif