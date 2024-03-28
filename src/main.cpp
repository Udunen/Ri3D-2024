/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\User                                             */
/*    Created:      Wed Mar 27 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include "joystick.cpp"

using namespace vex;

competition  Competition;

motor LeftDriveMotor = motor(1, ratio18_1, false);
motor RightDriveMotor = motor(2, ratio18_1, false);
motor FrontDriveMotor = motor(3, ratio18_1, false);
motor BackDriveMotor = motor(4, ratio18_1, false);
inertial Inertial = inertial(5);
motor arm = motor(7, ratio36_1, false);
motor deploy = motor(8, ratio18_1, false);
motor climbMotorA = motor(9, ratio18_1, false);
motor climbMotorB = motor(10, ratio18_1, true);
motor_group climb = motor_group(climbMotorA, climbMotorB);
controller Controller1 = controller(primary);


class Drives {
  public:
    void static robotOirented() {
      double yInput = getLeftY();
      double xInput = getLeftX();
      double turning = getTrueRightX();
      double leftMotor = yInput + turning;
      double rightMotor = yInput - turning;
      double frontMotor = xInput + turning;
      double backMotor = xInput - turning;

      LeftDriveMotor.spin(fwd, leftMotor, velocityUnits::pct);
      RightDriveMotor.spin(fwd, rightMotor, velocityUnits::pct);
      FrontDriveMotor.spin(fwd, frontMotor, velocityUnits::pct);
      BackDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }

    void static autonDrive(double xIn, double yIn, double turnIn) {
      double yInput = yIn;
      double xInput = xIn;
      double turning = turnIn;
      double leftMotor = yInput + turning;
      double rightMotor = yInput - turning;
      double frontMotor = xInput + turning;
      double backMotor = xInput - turning;

      LeftDriveMotor.spin(fwd, leftMotor, velocityUnits::pct);
      RightDriveMotor.spin(fwd, rightMotor, velocityUnits::pct);
      FrontDriveMotor.spin(fwd, frontMotor, velocityUnits::pct);
      BackDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }

    void static fieldOriented() {
      double headingRadians = Inertial.heading() * 3.141592/180;
      double yInput = getLeftY();
      double xInput = getLeftX();
      double sineHeading = sin(headingRadians);
      double cosHeading = cos(headingRadians);
      double rotatedYInput = xInput * sineHeading + yInput * cosHeading;
      double rotatedXInput = xInput * cosHeading - yInput * sineHeading;
      double turning = getTrueRightX();
      double leftMotor = rotatedYInput + turning;
      double rightMotor = -rotatedYInput + turning;
      double FrontMotor = rotatedXInput + turning;
      double backMotor = -rotatedXInput + turning;

      LeftDriveMotor.spin(fwd, leftMotor, velocityUnits::pct);
      RightDriveMotor.spin(fwd, rightMotor, velocityUnits::pct);
      FrontDriveMotor.spin(fwd, FrontMotor, velocityUnits::pct);
      BackDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }
};

void pre_auton(void) {
  Inertial.setHeading(0.0, degrees);
  Inertial.setRotation(0.0, degrees);
  Inertial.startCalibration();
  while(Inertial.isCalibrating()) {
    task::sleep(10);
  }
  Inertial.setHeading(0.0, degrees);
  Inertial.setRotation(0.0, degrees);

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

void usercontrol(void) {
  while(true) {
    Drives::fieldOriented();

    //zeroes inertial sensor
    if(Controller1.ButtonY.pressing()) {
      Inertial.setHeading(0.0, degrees);
      Inertial.setRotation(0.0, degrees);
      Inertial.startCalibration();
      while (Inertial.isCalibrating()) {
          task::sleep(10);
      }
      Inertial.setHeading(0.0, degrees);
      Inertial.setRotation(0.0, degrees);
    }

  }
}

void auton(void) {
  Drives::autonDrive(0.0, 0.5, 0);
  task::sleep(4000);
  Drives::autonDrive(0.0, 0.0, 0);
}

int main() {
  pre_auton();
  
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
   
  while(true) {
    task::sleep(10);
  }

}
