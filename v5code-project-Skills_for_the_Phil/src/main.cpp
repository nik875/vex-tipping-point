/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author                                                                  */
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FourBar              motor         20              
// BackClamp            motor         1               
// Clamp                motor         21              
// Conveyor             motor         16              
// Controller1          controller                    
// Controller2          controller                    
// Drivetrain           drivetrain    18, 5, 10, 2, 9 
// ---- END VEXCODE CONFIGURED DEVICES ----
 
#include "vex.h"
#include <math.h>
 
using namespace vex;
 
// A global instance of competition
competition Competition;
 
// define your global instances of motors and other devices here
double ROBOTCIRCUMFRENCE = 58.12;
double DEGREESPERINCH = 360 / 12.56;
int team = 0;
static double xPos = 11;
static double yPos = 1;
static double dir = 90;
void pre_auton(void)
{
 // I0nitializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.setStopping(brake);
 Brain.Screen.setFont(propXXL);
 Brain.Screen.setCursor(1,2); 
 Brain.Screen.setFillColor(transparent);
 Brain.Screen.setPenColor(green);
 Brain.Screen.setPenWidth(10);
 Brain.Screen.print("6096A");
 Brain.Screen.setCursor(1,11);
 Brain.Screen.setPenColor(red);
 Brain.Screen.print("6096A");
 Brain.Screen.setCursor(5,4);
 Brain.Screen.setPenColor(white);
 Brain.Screen.print("ROBORAYS1");
}


void bringBackClampDown()
{
   BackClamp.rotateFor(fwd, 1250, degrees, 200, velocityUnits::rpm);
}

void bringBackClampUp()
{
   BackClamp.startRotateFor(fwd, -1250, degrees, 200, velocityUnits::rpm);
}

void bringClampDown(){
  Clamp.spin(forward, 100, velocityUnits::rpm);
}

void bringClampUp(){
  Clamp.rotateFor(reverse, 165, degrees);
}

int stopDistance(int vel, int acc, int INTERVAL, int stopThresh) {
  int d = 0;
  for (; vel > stopThresh; vel -= acc)
    d += vel * INTERVAL;
  return d;
}

void move(double inches, int travelVel = 200, int acc = 10, int dec = 10, int stopThresh = 20) {
  int pos = 0; int INTERVAL = 100;
  for (int vel = 0; inches - pos > stopDistance(vel, dec, INTERVAL, stopThresh); vel += (vel + acc <= travelVel) ? vel + acc : travelVel) {
    Drivetrain.drive(forward, vel, rpm);
    wait(INTERVAL, seconds);
  }
  while (Drivetrain.velocity(rpm) > stopThresh) {
    Drivetrain.drive(forward, Drivetrain.velocity(rpm) - dec, rpm);
  }
  Drivetrain.stop(brake);
}
//red bottom autonomous/Blue Top
void autonomous(void) {
  wait(1, seconds);
  Drivetrain.setDriveVelocity(50,rpm);
  FourBar.setVelocity(100, rpm);
  Drivetrain.driveFor(reverse, 12, inches);
  BackClamp.spinFor(forward, 2290, degrees, 200, velocityUnits::rpm);
  Conveyor.spin(forward, 300, velocityUnits::rpm);
  Drivetrain.turnToHeading(85, degrees);
  Drivetrain.driveFor(forward, 24, inches);
  Drivetrain.turnToHeading(173, degrees);
  Drivetrain.driveFor(forward, 24, inches);
  wait(0.5, seconds);
  Clamp.spin(forward);
  wait(0.5, seconds);
  FourBar.spinFor(reverse, 826, degrees);
  Drivetrain.driveFor(forward, 45, inches);
  Drivetrain.turnToHeading(93, degrees);
  BackClamp.spinFor(forward, -500, degrees, 200, velocityUnits::rpm);
  Drivetrain.driveFor(forward, 30, inches); 
  Drivetrain.turnToHeading(180, degrees);
  Drivetrain.driveFor(forward, 9, inches);
  FourBar.spinFor(forward, 370, degrees);
  Clamp.spinFor(reverse, 100, degrees);
  wait(0.5, seconds);
  FourBar.spinFor(forward, -80, degrees);
  wait(0.5, seconds);

  //pushy stuff
  Drivetrain.driveFor(forward, -8.5, inches);
  FourBar.spinFor(540, degrees);
  Drivetrain.turnToHeading(355, degrees);
  Drivetrain.driveFor(reverse, -56, inches, 100, velocityUnits::rpm);
  Drivetrain.turnToHeading(125, degrees);
  Drivetrain.driveFor(reverse, -36, inches, 100, velocityUnits::rpm);
  Clamp.spin(forward);
  wait(1, seconds);
  Drivetrain.turnToHeading(160, degrees);
  Drivetrain.driveFor(reverse, -36, inches, 200, velocityUnits::rpm);
  Clamp.spinFor(reverse,100,degrees);
}


void usercontrol(void)
{
  Drivetrain.setDriveVelocity(200, rpm);
  Clamp.setTimeout(3,seconds);
  BackClamp.setTimeout(1, seconds);
  Drivetrain.setStopping(coast);
  FourBar.setStopping(brake);
  BackClamp.setStopping(hold);
 while (true)
 {
 
    // if ((Controller1.Axis3.value() < -2 && Controller1.Axis2.value() < -2) || (Controller1.Axis3.value() > 2 && Controller1.Axis2.value() > 2))
    // {
    //   if (abs(Controller1.Axis3.value() - Controller1.Axis2.value()) < 10)
    //   {   
    //     LB.spin(fwd, -Controller1.Axis3.value(), velocityUnits::pct);
    //     RB.spin(fwd, Controller1.Axis2.value(), velocityUnits::pct);
    //     LF.spin(fwd, -Controller1.Axis3.value(), velocityUnits::pct);
    //     RF.spin(fwd, Controller1.Axis2.value(), velocityUnits::pct);
    //   }
    //   else
    //   {
    //     LB.spin(fwd, -Controller1.Axis3.value(), pct);
    //     RB.spin(fwd, Controller1.Axis2.value(), pct);
    //     LF.spin(fwd, -Controller1.Axis3.value(), pct);
    //     RF.spin(fwd, Controller1.Axis2.value(), pct);
    //   }
    // }
    // else
    // {
    //   LB.spin(fwd, -Controller1.Axis3.value(),pct);
    //   RB.spin(fwd, Controller1.Axis2.value(), pct);
    //   LF.spin(fwd, -Controller1.Axis3.value(), pct);
    //   RF.spin(fwd, Controller1.Axis2.value(), pct);
    // }
   //Titler - L2
  
  //  if(Controller2.ButtonL2.pressing()) {

  //   Tilter.startRotateFor(reverse, 327.65, degrees);
    
  //  } else if(Controller2.ButtonL1.pressing()) {

  //   Tilter.startRotateFor(forward, 327.65, degrees);
  
  //  }
  //  if(Controller2.ButtonY.pressing()) {
  //   Tilter.startRotateFor(fwd, 500, degrees);
  // }

  
 

    //Clamp - R2 & R1
  if(Controller2.ButtonR2.pressing())
   {
     Clamp.spin(fwd, 200, velocityUnits::rpm); 

   }
   else if (Controller2.ButtonR1.pressing())
    {

      Clamp.startRotateFor(fwd, -165, degrees);
    }
    //BackClamp
    if(Controller2.ButtonL1.pressing())
    {
      BackClamp.startRotateFor(fwd, -2051, degrees, 200, velocityUnits::rpm);
    }
    else if(Controller2.ButtonL2.pressing())
   {
     BackClamp.startRotateFor(fwd, 2051, degrees, 200, velocityUnits::rpm);
    
   }

         

    //Conveyer - A
   if(Controller2.ButtonA.pressing()){

     Conveyor.spin(fwd, 160 ,velocityUnits::rpm);

   } else if(Controller2.ButtonX.pressing()){

     Conveyor.spin(reverse, 0 ,velocityUnits::pct);
   } 

    //FourBar - Axis 2 (Right Joystick) 
    if(Controller2.Axis2.value() > 2 || Controller2.Axis2.value() < -2) {
      FourBar.spin(forward, -Controller2.Axis2.value(), velocityUnits::pct);
    }
     else 
    {
      FourBar.spin(forward, 0, velocityUnits::pct);

    }
   

   
  }
 }

int main(){
 Drivetrain.setDriveVelocity(200,rpm);
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
 // Run the pre-autonomous function.
 pre_auton();
 // Prevent main from exiting with an infinite loop.
}


/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
 
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LB                   motor         20             
// RB                   motor         10             
// LF                   motor         11             
// RF                   motor         1              
// LIn                  motor         12             
// RIn                  motor         2              
// Controller1          controller                   
// Controller2          controller                   
// ConveyerMotor        motor         8              
// Strafe               motor         3              
// Vision1              vision        4              
// Optical              optical       7              
// Distance             distance      17             