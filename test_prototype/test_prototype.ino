#include "Navigation.h"

#include <vector>

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Rover::init();
}

void straight() {
  Rover::forwards(60);
  delay(20000);
  Rover::stop();
}

void loop() {

  //sensor reading code
  
  //Rover::leftRotate(35);
  /*
  while(true) {
    int start = millis();
    Rover::tickGyro();
    int elapsed = millis() - start;
    float turnAngle = Rover::turnAngle();
    float tiltAngle = Rover::tiltAngle();

    Serial.print("turn: ");
    Serial.print(turnAngle);
    Serial.print(" tilt: ");
    Serial.print(tiltAngle);
    Serial.print(": ");
    Serial.print(elapsed);
    Serial.print("ms.     ");

    // front distance (ultrasonic) reading code
    
    int dist = Rover::distanceFrontLaser();
    //if (dist < 2047 && dist > 100) {
      Serial.print(dist);
      Serial.println(" mm");
    //}
  }*/


  //gameday

  while(Rover::distanceFrontLaser() > 1000 || Rover::distanceFrontLaser() < 100);
  Navigation::goToWall();
  Navigation::climbUpWall();
  Navigation::traverseWall();
  Navigation::climbDownWall();

  delay(500);
  Rover::resetTurn();
  //Navigation::hardcodedSearch(135.0f, 1.0f);
  Navigation::quadrantSearch(1);
  while(true);

}
