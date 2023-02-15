#include "Navigation.h"

#include <vector>

namespace Navigation {

bool goToWall() {
  // travel straight to the wall
  Rover::forwards(100);
  while(Rover::distanceFront() > 5);
  Rover::stop();
}

bool climbUpWall() {
  // slow down when getting onto the wall
  Rover::forwards(60); 
  while(!Rover::isVertical());

  // on the wall vertically now, can go full speed until on the top of the wall
  Rover::forwards(100);
  while(Rover::isVertical());

  // 
}

bool traverseWall() {
  // now at the top of the wall, not vertical anymore, go slow
  Rover::forwards(40);
  while(!Rover::isVertical());
}

bool climbDownWall() {
  // vertically on the wall again facing down, can go full speed until near the ground
  Rover::forwards(100);
  while(Rover::distanceFront() > 10);
  // slow down to get off the wall
  Rover::forwards(40);
  while(Rover::isVertical());
}

bool search() {
  long leftDist;
  long rightDist;
  std::vector<float> anglesOfInterest;
  // rotate left 90 degrees
  Rover::leftRotate(100);
  while(Rover::turnAngle() > -90) {
    // needs to tick the gyro as frequently as possible here
    Rover::tickGyro();
  }
  Rover::stop();
  leftDist = Rover::distanceFront();
  
  // rotate right 180 degrees, do scan for points of interest
  long prevDistMeasurement = leftDist;
  long threshold = 40;
  Rover::rightRotate(50);
  while(Rover::turnAngle() < 90) {
    // needs to tick the gyro as frequently as possible here
    Rover::tickGyro();
    long dist = Rover::distanceFront();
    if (abs(prevDistMeasurement - dist) > threshold) {
      anglesOfInterest.push_back(Rover::turnAngle());
    }
  }

  // note that there should be an even number of "angles of interest" since any object will have 2 angles
  // first is when the scan first sees the object, second is when the scan no longer sees the object


  Rover::stop();
  rightDist = Rover::distanceFront();

  // visit points of interest
  for(int i = 0; (i + 1) < anglesOfInterest.size(); i += 2) {
    float angle = (anglesOfInterest[i] + anglesOfInterest[i + 1]) / 2;
    Rover::leftRotate(50);
    while(Rover::turnAngle() > angle) {
      Rover::tickGyro();
    }
    Rover::stop();
    Rover::forwards(100);
    long currentTime = millis();
    while(Rover::distanceFront() > 5);
    long elapsedTime = millis() - currentTime;
    Rover::stop();

    // if at the post, break
    if (Rover::atDestination()) break;
    else {
      Rover::backwards(100);
      long currentTime2 = millis();
      while(millis() - currentTime2 < elapsedTime);
      Rover::stop();
    }
  }
}

}