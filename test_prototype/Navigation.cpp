#include "Navigation.h"


namespace Navigation {

void goToWall() {
  // travel straight to the wall
  Rover::forwards(90);
  while(Rover::distanceFrontLaser() > 150);
  Rover::stop();
}

void climbUpWall() {
  // slow down when getting onto the wall
  Rover::forwards(70); 
  while(Rover::distanceFrontLaser() < 2000);
  Rover::backwards(40);
  delay(800);
  Rover::forwards(70);
  delay(4000);
  while(Rover::distanceFrontLaser() > 2000 || Rover::distanceFrontLaser() < 1700);

}

void traverseWall() {
  // now near top of the wall, go fast to try and go over
  delay(700);
  Rover::forwards(80);
  delay(500);
  Rover::stop();
}

void climbDownWall() {
  // vertically on the wall again facing down
  Rover::stop();
  Rover::forwards(70);
  delay(600);
  while(Rover::distanceFrontLaser() > 150);
  while(Rover::distanceFrontLaser() < 300);
  Rover::stop();
}

void hardcodedSearch(float angle, float distance) {
  // angle is measured from wall CCW
  if (angle < 90) {
    float convertedAngle = -(90 - angle);
    if (Rover::turnAngle() != convertedAngle) Rover::rightRotate(35);
    while(Rover::turnAngle() > convertedAngle) {
      Rover::tickGyro();
    }
    Rover::stop();
  } else {
    float convertedAngle = angle - 90;
    if (Rover::turnAngle() != convertedAngle) Rover::leftRotate(35);
    while(Rover::turnAngle() < convertedAngle) {
      Rover::tickGyro();
    }
    Rover::stop();
  }
  // 0.22m/s at 80 speed
  Rover::forwards(80);
  Serial.println(1000 * (distance/0.24f));
  delay(1000 * (distance/0.24f));
  Rover::stop();
}


void quadrantSearch(uint8_t quadrant) {
  // 1 2
  // 3 4
  // ---


  int foundDistance = 0;

  if (quadrant == 1) {
    // center of area
    Rover::forwards(80);
    Serial.println(1000 * (0.70f/0.24f));
    delay(1000 * (0.70f/0.24f));
    Rover::stop();
    delay(500);

    Rover::resetTurn();

    Rover::leftRotate(35); 
    bool found = false;
    while(Rover::turnAngle() < 90) {
      Rover::tickGyro();
      long dist = Rover::distanceFrontLaser();
      if (dist < 1000 && dist > 100) {
        found = true;
        foundDistance = dist;
        delay(200);
        break;
      }
    }
    Rover::stop();
    if (!found) {
      delay(500);
      Rover::resetTurn();
      Rover::rightRotate(35);
      while(Rover::turnAngle() > -45) {
        Rover::tickGyro();
      }
      foundDistance = 1000;
      Rover::stop();  
    }
  } else if (quadrant == 2) {
    // center of area
    Rover::forwards(80);
    Serial.println(1000 * (0.70f/0.24f));
    delay(1000 * (0.70f/0.24f));
    Rover::stop();
    delay(500);

    Rover::resetTurn();
    Rover::rightRotate(35);
    bool found = false;
    while(Rover::turnAngle() > -90) {
      Rover::tickGyro();
      long dist = Rover::distanceFrontLaser();
      if (dist < 1000 && dist > 100) {
        found = true;
        foundDistance = dist;
        break;
      }
    }
    Rover::stop();
    if (!found) {
      delay(500);
      Rover::resetTurn();
      Rover::leftRotate(35);
      while(Rover::turnAngle() < 45) {
        Rover::tickGyro();
      }
      foundDistance = 1000;
      Rover::stop();  
    }
  } else if (quadrant == 3) {
    Rover::resetTurn();
    Rover::leftRotate(35);
    bool found = false;
    while(Rover::turnAngle() < 90) {
      Rover::tickGyro();
      long dist = Rover::distanceFrontLaser();
      if (dist < 1300 && dist > 100) {
        Rover::stop();
        found = true;
        foundDistance = dist;
        break;
      }
    }
    Rover::stop();
    if (!found) {
      delay(500);
      Rover::resetTurn();
      Rover::rightRotate(35);
      while(Rover::turnAngle() > -45) {
        Rover::tickGyro();
      }
      foundDistance = 1300;
      Rover::stop();  
    }
  } else if (quadrant == 4) {
    Rover::forwards(80);
    Serial.println(1000 * (0.8f/0.24f));
    delay(1000 * (0.8f/0.24f));
    Rover::stop();
    delay(500);

    Rover::resetTurn();
    Rover::rightRotate(35);
    bool found = false;
    while(Rover::turnAngle() > -90) {
      Rover::tickGyro();
      long dist = Rover::distanceFrontLaser();
      if (dist < 700 && dist > 100) {
        Rover::stop();
        found = true;
        foundDistance = dist;
        break;
      }
    }
    Rover::stop();
    if (!found) {
      delay(500);
      Rover::resetTurn();
      Rover::leftRotate(35);
      while(Rover::turnAngle() < 45) {
        Rover::tickGyro();
      }
      foundDistance = 700;
      Rover::stop();  
    }
  }


  // move towards pole
  Rover::forwards(90);
  delay((int)(((float)foundDistance) / 0.24f));
  //while(Rover::distanceFrontLaser() > 150);
  Rover::stop();

}

}