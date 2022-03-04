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
// LB                   motor         1               
// RB                   motor         2               
// LF                   motor         3               
// RF                   motor         4               
// ConveyerMotor        motor         6               
// Controller1          controller                    
// Controller2          controller                    
// Vision               vision        7               
// EncoderA             encoder       C, D            
// Claw                 motor         5               
// Tilter               motor         8               
// FourBar              motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----
 
#include "vex.h"
 
using namespace vex;
 
// A global instance of competition
competition Competition;
 
// define your global instances of motors and other devices here
int team = 0;
void pre_auton(void)
{
 // Initializing Robot Configuration. DO NOT REMOVE!
 vexcodeInit();
 EncoderA.setPosition(0, degrees);
}
 void moveForward(double distance)
 {
 
 }
 
 void moveBackward(double distance)
 {
 
 }

 void turnRight()
 {

 }

 void turnLeft()
 {

 }
 
void moveUp()
{

}

void moveOut()
{
  
}

//red bottom autonomous/Blue Top
void autonomous(void) {

}
 
void usercontrol(void)
{
 bool yPressed = false;
 bool autoEject = false;
 EncoderA.setPosition(0, degrees);

 while (true)
 {
   //Drive
   if ((Controller1.Axis3.value() < -2 && Controller1.Axis2.value() < -2) || (Controller1.Axis3.value() > 2 && Controller1.Axis2.value() > 2))
   {
     if (abs(Controller1.Axis3.value() - Controller1.Axis2.value()) < 10)
     {  
       LB.spin(fwd, Controller1.Axis3.value() * 1 / 2, velocityUnits::pct);
       RB.spin(fwd, Controller1.Axis2.value() * 1 / 2, velocityUnits::pct);
       LF.spin(fwd, Controller1.Axis3.value() * 1 / 2, velocityUnits::pct);
       RF.spin(fwd, Controller1.Axis2.value() * 1 / 2, velocityUnits::pct);
     }
    
    else
    {
      LB.spin(fwd, Controller1.Axis3.value() * 1 / 2, pct);
      RB.spin(fwd, Controller1.Axis2.value() * 1 / 2, pct);
      LF.spin(fwd, Controller1.Axis3.value() * 1 / 2, pct);
      RF.spin(fwd, Controller1.Axis2.value() * 1 / 2, pct);
    }
   }
   //Titler - L2
   bool up = false;

   if(Controller2.ButtonL2.pressing() && up == false) {

    Tilter.spinFor(forward, 5, degrees);

   } else if(Controller2.ButtonL2.pressing() && up == true) {

    Tilter.spinFor(reverse, 5, degrees);
   }
    //Claw - R2
    bool holding = false;
   if(Controller2.ButtonR2.pressing() && holding == false){

     Claw.spinFor(forward, 5, degrees);
     holding = true;

   } else if(Controller2.ButtonR2.pressing() && holding == true){

     Claw.spinFor(reverse, 5, degrees);
     holding = false;
   } 
    //Conveyer - A
     bool running = false;
   if(Controller2.ButtonR2.pressing() && running == false){

     ConveyerMotor.spinFor(forward, 5, degrees);
     running = true;

   } else if(Controller2.ButtonR2.pressing() && running == true){

     ConveyerMotor.spinFor(reverse, 5, degrees);
     running = false;
   } 

    //FourBar - Axis 2 (Right Joystick) 
    if(Controller2.Axis2.value() != 0){

     FourBar.spin(fwd, Controller2.Axis2.value() *0.5 , velocityUnits::pct);

    }
   

   
  }
 }

int main(){

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
