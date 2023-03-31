#ifndef NAVIGATION
#define NAVIGATION

#include "Rover.h"

#include <vector>

namespace Navigation {

void goToWall();          // get to the steel portion of the wall
void climbUpWall();       // get onto the wall vertically and ascend to the top
void traverseWall();      // controls for getting over the bump/top of the wall
void climbDownWall();     // descending the wall and getting back onto the ground
void hardcodedSearch(float angle, float distance);
void quadrantSearch(uint8_t quadrant);

}

#endif