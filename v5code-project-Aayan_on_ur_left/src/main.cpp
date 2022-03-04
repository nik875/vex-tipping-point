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
  calibrateDrivetrain();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Drivetrain Calibrated");
}

void matchload() {
  int x = 1;
  if (x<= 6){
    Drivetrain.driveFor(forward, 5, inches);
    Drivetrain.driveFor(reverse, 5, inches);
    x += 1;
  }
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
  
  // Conveyor.setVelocity(300,rpm);
  // BackClamp.setVelocity(200, rpm);
  // Drivetrain.setStopping(coast);
  // Drivetrain.setDriveVelocity(200, rpm);
  // Clamp.setVelocity(200, rpm); 
  // FourBar.setVelocity(100, rpm); 
  // //Drivetrain.driveFor(forward, 40, inches);
  
  
  move(1000);
  Drivetrain.driveFor(forward, 2, inches, 50, velocityUnits::rpm);
  Clamp.spin(forward, 200, velocityUnits::rpm);
  wait(200, msec);
  Drivetrain.driveFor(reverse, 43, inches, 150, velocityUnits::rpm);
  Clamp.spinFor(reverse,100, degrees);
  Drivetrain.turnToHeading(260, degrees);
  FourBar.spinFor(reverse,600, degrees, 200, velocityUnits::rpm);
  Drivetrain.driveFor(reverse, 12, inches, 100, velocityUnits::rpm);
  BackClamp.spinFor(forward,2290, degrees, 150, velocityUnits::rpm);
  Drivetrain.driveFor(forward, 5, inches, 150, velocityUnits::rpm);
  Conveyor.spin(forward, 300, velocityUnits::rpm);
  Drivetrain.driveFor(forward, 10, inches, 50, velocityUnits::rpm);
  BackClamp.spinFor(reverse, 2290, degrees, 150, velocityUnits::rpm);
  Drivetrain.driveFor(forward, 5, inches, 25, velocityUnits::rpm);
  


  // wait(200, msec);
  // Drivetrain.setDriveVelocity(30, rpm);
  // wait(0.5,seconds);
  // Drivetrain.driveFor(forward, 5, inches);
  
  // wait(500, msec);
  // Clamp.spin(forward);
  // wait(100, msec);
  // //Drivetrain.turnFor(right, 2, degrees);
  // Drivetrain.setDriveVelocity(200, percent);
  // Drivetrain.driveFor(reverse,46.5, inches);
  // Drivetrain.turnToHeading(262, degrees);
  // Drivetrain.driveFor(reverse,9, inches);
  // BackClamp.spinFor(forward,2245.1, degrees);
  // Drivetrain.driveFor(forward, 2, inches);
  // Conveyor.spin(forward);
  // FourBar.spinFor(reverse,1000, degrees, 100, velocityUnits::rpm);
  // Drivetrain.driveFor(forward,16, inches, 50, velocityUnits::rpm);
}

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  while (true) {
     
     if(Controller1.ButtonB.pressing()){
       FourBar.spinFor(reverse, 400, degrees);
     }
    wait(20, msec);
  }
  Drivetrain.setDriveVelocity(Controller1.Axis3.position(), percent);
  Drivetrain.setDriveVelocity(Controller1.Axis2.position(), percent);
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
