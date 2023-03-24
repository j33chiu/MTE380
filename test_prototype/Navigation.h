#ifndef NAVIGATION
#define NAVIGATION

#include "Rover.h"

#include <vector>

namespace Navigation {

bool goToWall();          // get to the steel portion of the wall
bool climbUpWall();       // get onto the wall vertically and ascend to the top
bool traverseWall();      // controls for getting over the bump/top of the wall
bool climbDownWall();     // descending the wall and getting back onto the ground
void searchDebug();
bool search();            // find the target

}

#endif