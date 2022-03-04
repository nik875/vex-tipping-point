// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
}

void driveInt(int vel, float i) {
  Drivetrain.setDriveVelocity(vel, rpm);
  Drivetrain.drive(forward);
  // if (vel > 0) {
  //   Drivetrain.drive(forward);
  // }
  // else {
  //   Drivetrain.drive(reverse);
  // }
  wait(i, seconds);
}

void move(int deg, int acc = 50) {
  Drivetrain.setStopping(brake);
  leftMotorA.setPosition(0, degrees);
  float INTERVAL = .1;
  int sign = (deg / abs(deg));
  int vel = 0;
  auto stopDistance = [=, &vel, &acc] {
    int d = 0;
    for (int i = vel; abs(i) > 0; i -= sign * acc)
      d += (i / 60) * 360 * INTERVAL;
    return d;
  };
  while (abs((int) leftMotorA.position(degrees) - deg) > stopDistance()) {
    vel = sign * (abs(vel) + acc <= 200) * acc + vel;
    driveInt(vel, INTERVAL);
  }
  while (vel > 0) {
    vel -= sign * acc;
    driveInt(vel, INTERVAL);
  }
  Drivetrain.stop();
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  BackClamp.setTimeout(2500, msec);
  move(1000);
  Drivetrain.driveFor(forward, 2, inches, 50, velocityUnits::rpm);
  Clamp.spin(forward, 200, velocityUnits::rpm);
  wait(200, msec);
  Drivetrain.driveFor(reverse, 6, inches, 175, velocityUnits::rpm);
   Drivetrain.turnToHeading(249.9, degrees);
  Drivetrain.driveFor(reverse, 34, inches, 126, velocityUnits::rpm);
  BackClamp.spinFor(forward, 1150, degrees, 200, velocityUnits::rpm);
  Drivetrain.turnFor(left, 20, degrees, 200, velocityUnits::rpm);
  Drivetrain.driveFor(forward, 50, inches, 200, velocityUnits::rpm);
  BackClamp.spinFor(reverse, 1150, degrees, 200, velocityUnits::rpm);

}

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  Drivetrain.setDriveVelocity(Controller1.Axis3.position(), percent);
  Drivetrain.setDriveVelocity(Controller1.Axis2.position(), percent);
  while (true) {
     //Clamp-R2&R1
     if(Controller2.ButtonR2.pressing()){
       Clamp.spin(fwd, 200, velocityUnits::rpm);
     }
     else if (Controller2.ButtonR1.pressing()){
       Clamp.startRotateFor(fwd, -165, degrees);
     }
     //BackClamp
     if(Controller2.ButtonL1.pressing()){
       BackClamp.startRotateFor(fwd, -2290, degrees, 200, velocityUnits::rpm);
     }
     else if(Controller2.ButtonL2.pressing()){
       BackClamp.startRotateFor(fwd, 2290, degrees, 200, velocityUnits::rpm);
     }
     //Conveyor
     if(Controller2.ButtonA.pressing()){
       Conveyor.spin(fwd, 160, velocityUnits::rpm);
     }
     else if(Controller2.ButtonX.pressing()){
       Conveyor.spin(reverse,0, velocityUnits::pct);
     }
    wait(20, msec);
  }
}
int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}