#include "Navigation.h"


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Rover::init();
}

void loop() {
  // prototype: show movement

  Serial.println("Program started");
  // test forwards then backwards with stop in between
  Serial.println("rover forwards");
  Rover::forwards(80);
  delay(2000);
  Serial.println("rover stop");
  Rover::stop();
  delay(2000);
  Serial.println("rover backwards");
  Rover::backwards(80);
  delay(2000);
  Serial.println("rover right rotate");
  Rover::rightRotate(50);
  delay(3000);
  Serial.println("rover left rotate");
  Rover::leftRotate(50);
  delay(3000);
  Rover::stop();
  while(true) {
    // prototype: show sensor works
    Serial.println(Rover::distanceFront());
    delay(10);
  }

}
