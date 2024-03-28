#include "vex.h"
#include <cmath>


/* 
  this class was made by Cole Huffine class of 2024
  lets hope rivera gives this to future classes bc
  i think it would be useful for everyone
*/

/*
  In order to use this class, put it in your "include" folder
  and then put '#inlcude "joystick.cpp"' at the top of your main file

  Usage:
    If you need to get the value of a joystick fixed to account for the physical limitations
      getLeftX();
      getLeftY();
      getRightX();
      getRightY();

    If you need to get the REAL value of a joystick, as inputted into the brain
      getTrueLeftX();
      getTrueLeftY();
      getTrueRightX();
      getTrueRightY();
*/

/*
  If you want to use a second controller, or name your controller something else other than Controller1,
  then go to all of the getTrue methods, and change the name to what you have as your config
*/


//this returns the sign of the value you pass in as a parameter
//i originally made this file in java, and c++ doesnt have this
//built in so i made it :)
int signum(double val) {
  return (val >= 0) ? 1 : -1;
}


//use these if you need to get the true value of any joystick
double getTrueLeftX() {
  return Controller1.Axis4.position(pct);
}

double getTrueLeftY() {
  return Controller1.Axis3.position(pct);
}

double getTrueRightX() {
  return Controller1.Axis1.position(pct);
}

double getTrueRightY() {
  return Controller1.Axis2.position(pct);
}

/*
  * This method is to fix the flawed input of a vex controller. The input is a box,
  * both X and Y axis can go from -1 to 1. However, since the controller build physically
  * forces the joystick into a circle, its impossible to reach the the corner of the box.
  * This method fixes that issue.
*/  
double joystickFix(bool isLeft, bool isX) {
  double root2 = sqrt(2);
  double x = isLeft ? getTrueLeftX() : getTrueRightX();
  double y = isLeft ? getTrueLeftY() : getTrueRightY();
  double magnitude = sqrt(x*x + y*y);
  double values[2] = {
    signum(x) * fmin(fabs(x*root2), magnitude),
    signum(y) * fmin(fabs(y*root2), magnitude)
  };
  return isX? values[0] : values[1];
}


// these return the fixed value of all of the joysticks
double getLeftX() {
  return joystickFix(true, true);
}

double getLeftY() {
  return joystickFix(true, false);
}

double getRightX() {
  return joystickFix(false, true);
}

double getRightY() {
  return joystickFix(false, false);
}


