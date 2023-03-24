#include "Navigation.h"

#include <vector>

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Rover::init();
}

void TP1() {
  // ground speed
  delay(2000);
  Rover::forwards(80);
  delay(20000);
  Rover::stop();
  while(true);
}

void TP2() {
  // wall speed
  delay(2000);
  Rover::forwards(60);
  delay(10000);
  Rover::stop();
  while(true);
}

void TP3(int distance) {
  // distance sensor
  delay(2000);
  Rover::forwards(70);

  int dummy = Rover::distanceFrontLaser();
  while(dummy == 0) {
    int poll = Rover::distanceFrontLaser();
    if (poll > 400) dummy = poll;
  }
  Serial.println(dummy);
  while(dummy > (distance + 100)) {
    int poll = Rover::distanceFrontLaser();
    if (poll > 400) dummy = poll;
    Serial.println(dummy);
  }
  Rover::stop();
}

void loop() {
  //TP1();
  //TP2();
  TP3(1500);
  delay(10000);
  TP3(500);
  while(true);

  //int dist = Rover::distanceFront();
  //Serial.println(dist);

  // turn with gyro code
  /*
  Rover::leftRotate(35);
  while(Rover::turnAngle() < 90) {
    Rover::tickGyro();
  }
  Rover::stop();

  delay(5000);
  Rover::rightRotate(35);
  while(Rover::turnAngle() > -90) {
    Rover::tickGyro();
  }
  Rover::stop();

  while(true);
  */

  //gyro reading code
  /*
  int start = millis();
  Rover::tickGyro();
  int elapsed = millis() - start;
  float turnAngle = Rover::turnAngle();*/
/*
  Serial.print(turnAngle);
  Serial.print(": ");
  Serial.print(elapsed);
  Serial.print("ms.     ");*/

  // front distance (ultrasonic) reading code
  
  int dist = Rover::distanceFrontLaser();
  if (dist < 2047 && dist > 100) {
    Serial.print(dist);
    Serial.println(" mm");
  }

  //
  //Navigation::searchDebug();

  //while(true);

}
