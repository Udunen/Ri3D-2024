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

motor leftFrontDriveMotor = motor(PORT17, ratio18_1, false);
motor leftBackDriveMotor = motor(PORT19, ratio18_1, false);
motor rightFrontDriveMotor = motor(PORT7, ratio18_1, false);
motor rightBackDriveMotor = motor(PORT8, ratio18_1, false);
motor frontDriveMotor = motor(PORT9, ratio18_1, true);
motor backDriveMotor = motor(PORT20, ratio18_1, false);
inertial inertialSens = inertial(PORT4);
motor arm = motor(PORT1, ratio36_1, false);
motor deploy = motor(PORT2, ratio18_1, false);
motor climbMotorA = motor(PORT6, ratio36_1, false);
motor climbMotorB = motor(PORT16, ratio36_1, true);
motor_group climb = motor_group(climbMotorA, climbMotorB);
motor_group leftDrive = motor_group(leftFrontDriveMotor, leftBackDriveMotor);
motor_group rightDrive = motor_group(rightFrontDriveMotor, rightBackDriveMotor);
//using the climb group to run the new arm group to have two different motor carts run in tandem
controller con1 = controller(primary);


class Drives {
  public:
    void static robotOriented(double xIn, double yIn, double turnIn) {
      double leftMotor = yIn + turnIn;
      double rightMotor = -yIn + turnIn;
      double frontMotor = xIn + turnIn;
      double backMotor = -xIn + turnIn;

      leftDrive.spin(fwd, leftMotor, velocityUnits::pct);
      rightDrive.spin(fwd, rightMotor, velocityUnits::pct);
      frontDriveMotor.spin(fwd, frontMotor, velocityUnits::pct);
      backDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }

    void static fieldOriented() {
      double headingRadians = inertialSens.heading() * 3.141592/180;
      double yInput = getLeftY(con1, 5);
      double xInput = getLeftX(con1, 5);
      double sineHeading = sin(headingRadians);
      double cosHeading = cos(headingRadians);
      double rotatedYInput = xInput * sineHeading + yInput * cosHeading;
      double rotatedXInput = xInput * cosHeading - yInput * sineHeading;
      double turning = getTrueRightX(con1, 5);
      double leftMotor = rotatedYInput + turning;
      double rightMotor = -rotatedYInput + turning;
      double FrontMotor = rotatedXInput + turning;
      double backMotor = -rotatedXInput + turning;

      leftDrive.spin(fwd, leftMotor, velocityUnits::pct);
      rightDrive.spin(fwd, rightMotor, velocityUnits::pct);
      frontDriveMotor.spin(fwd, FrontMotor, velocityUnits::pct);
      backDriveMotor.spin(fwd, backMotor, velocityUnits::pct);
    }
};

void usercontrol(void) {
  inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);
  inertialSens.startCalibration();
  while(inertialSens.isCalibrating()) {
    task::sleep(10);
  }
  inertialSens.setHeading(0.0, degrees);
  inertialSens.setRotation(0.0, degrees);
  while(true) {
    Drives::fieldOriented();
    //zeroes inertial sensor
    if(con1.ButtonY.pressing()) {
      inertialSens.setHeading(0.0, degrees);
      inertialSens.setRotation(0.0, degrees);
      inertialSens.startCalibration();
      while (inertialSens.isCalibrating()) {
          task::sleep(10);
      }
      inertialSens.setHeading(0.0, degrees);
      inertialSens.setRotation(0.0, degrees);
    }
    if(con1.ButtonLeft.pressing()) {
      climbMotorA.setStopping(brake);
      climbMotorB.setStopping(brake);
    }
    if(con1.ButtonRight.pressing()) {
      climbMotorA.setStopping(hold);
      climbMotorB.setStopping(hold);
    }
    if(con1.ButtonUp.pressing()) {
      climb.setStopping(hold);
    }
    if(con1.ButtonDown.pressing()) {
      climb.setStopping(coast);
    }
    if (con1.ButtonR1.pressing()) {
      climb.spin(forward);
      climb.setStopping(hold);
    } else if (con1.ButtonR2.pressing()) {
      climb.spin(reverse);
      climb.setStopping(hold);
    } else  {
      climb.stop();
    }
    wait(20,msec);
    }
  }


void auton(void) {
  Drives::robotOriented(0.0, 50, 0);
  task::sleep(4000);
  Drives::robotOriented(0.0, 0.0, 0);
}

int main() {
  climb.setMaxTorque(100, pct);
  climb.setVelocity(100, pct);
  climbMotorA.setVelocity(100, pct);
  climbMotorB.setVelocity(100, pct);
  climbMotorA.setMaxTorque(100, pct);
  climbMotorB.setMaxTorque(100, pct);
  climb.setStopping(coast);

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
   
  while(true) {
    task::sleep(10);
  }

}
