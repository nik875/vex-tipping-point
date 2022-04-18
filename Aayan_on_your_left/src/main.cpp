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
// BackClampLimit       bumper        A
// ---- END VEXCODE CONFIGURED DEVICES ----

/*//////////////////////////////////////////////////////////////////////////

BUILTINS FOR COMPETITION FORMAT

//////////////////////////////////////////////////////////////////////////*/

#include "vex.h"
#include <math.h>
#include "VisionSensor.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double ROBOTCIRCUMFRENCE = 58.12;
double DEGREESPERINCH = 360 / 12.56;

/*//////////////////////////////////////////////////////////////////////////

PRE-AUTON (INCLUDING ALL FUNCTIONS)

//////////////////////////////////////////////////////////////////////////*/

void pre_auton(void) {
  // I0nitializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Drivetrain.setStopping(brake);
  Brain.Screen.setFont(propXXL);
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.setPenColor(green);
  Brain.Screen.setPenWidth(10);
  Brain.Screen.print("6096A");
  Brain.Screen.setCursor(1, 11);
  Brain.Screen.setPenColor(red);
  Brain.Screen.print("6096A");
  Brain.Screen.setCursor(5, 4);
  Brain.Screen.setPenColor(white);
  Brain.Screen.print("ROBORAYS1");
  Drivetrain.drive(forward, 1, rpm);
}

/*//////////////////////////////////////////////////////////////////////////

AUTONOMOUS (INCLUDING ALL FUNCTIONS)

//////////////////////////////////////////////////////////////////////////*/
//regular functions

void BackFortnite() {
backClamp1.set(true);
backClamp2.set(true);
}

void BackFortnite2(){
  backClamp1.set(false);
  backClamp2.set(false);
}

//PID FUNCTIONS


// OPTIMIZE
int timestep = 10;  // Delay for PID, in ms
// OPTIMIZE
double k = 10;  // Maximum amount drivetrain may increase/decrease power for PID
// OPTIMIZE
double kVis = 10;  // Maximum amount vision sensor may increase/decrease power
int maxVel = 600 - k;  // Maximum velocity that we're allowed to move
// OPTIMIZE
double ACC_RATE = 5;  // Maximum amount of acceleration (rpm / timestep)
double vel = 0;  // Velocity for PID (reset in move)
int targetPos = 0;  // Position we should be at in PID
// OPTIMIZE
int DEC_RATE = 5;  // Rate of decelleration with braking scheme (rpm / timestep)

int visionSample(vision::signature sig, int n) {
  int sum = 0;
  for (int i = 0; i < n; i ++) {
    AllSeeingEye.takeSnapshot(sig);
    sum += AllSeeingEye.largestObject.centerX;
  }
  return sum / n;
}

double average(double a, double b) {  // Average of two numbers
  return (a + b) / 2;
}

double degreesToDistance(double deg) {  // Convert a degree value of an encoder to a distance
  return deg / DEGREESPERINCH;
}

double distanceToDegrees(double dist) {  // Convert a distance in inches to a degree value
  return dist * DEGREESPERINCH;
}

double correct(double lead) {  // Converts a lead in inches to a velocity correction
  return tanh(lead) * k;
}

double visionCorrect(int x) {  // Converts a vision sensor deviation from target center x to left drive correction
  return x / 128 * kVis;
}

double stopDistance() {  // Calculate how much distance we need to fully stop the robot
  // Simulate a stopping action
  int d = 0;  // Distance we travelled so far
  int v = average(leftEncoder.velocity(rpm), rightEncoder.velocity(rpm));
  while (v > 0) {
    d += degreesToDistance(v * 360 * timestep / (100 * 60));
    v -= DEC_RATE;
  }
  return d;
}

void stop() {  // Custom braking scheme
  int v = Drivetrain.velocity(rpm);
  while (average(leftEncoder.velocity(rpm), rightEncoder.velocity(rpm)) > 0) {
    v -= DEC_RATE;
    Drivetrain.drive(forward, v - DEC_RATE, rpm);
    wait(timestep, msec);
  }
}

double alignNeutral() {  // Correct motion to align to neutral goal
  return visionCorrect(visionSample(NEUTRAL, 100) - 128);
}

double alignRed() {  // Correct motion to align to red alliance goal
  return visionCorrect(visionSample(RED_ALLIANCE, 100) - 128);
}

double alignBlue() {  // Correct motion to align to blue alliance goal
  return visionCorrect(visionSample(BLUE_ALLIANCE, 100) - 128);
}

// Drive forward a given distance with PID and acceleration curve
// Optional correction function which can modify drivetrain
void move(double dist, double (*correctionFunc)() = []()->double{return 0;}) {
  // dist = distanceToDegrees(dist);  // Convert inches to degrees
  // Reset encoder positions
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  vel = 0;  // Reset velocity
  targetPos = 0;  // Reset target position
  while (dist - degreesToDistance(  // While we have enough distance to stop
      abs(average(
        leftEncoder.position(degrees), rightEncoder.position(degrees)*-1)
      )) > stopDistance()) {
    double leftLead = degreesToDistance(leftEncoder.position(degrees)) - targetPos;
    double rightLead = abs(degreesToDistance(rightEncoder.position(degrees)*-1)) - targetPos;
    double leftVel = vel + correct(leftLead) + correctionFunc() / 2;  // Divide correctionFunc() by 2 to equally correct both sides
    double rightVel = vel + correct(rightLead) - correctionFunc() / 2;
    targetPos += average(leftVel, rightVel) * timestep;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(targetPos);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(leftEncoder.position(degrees));
    leftDrive.spin(forward, leftVel, rpm);
    rightDrive.spin(forward, rightVel, rpm);
    if (vel + ACC_RATE < maxVel)
      vel += ACC_RATE;
    else
      vel = maxVel;
    wait(timestep, msec);
  }
  stop();
}

void autonomous(void) {
   /*///////////////////////////////////
    1 DEGREE = 0.0239982 inches

    1 INCH = 41.6697919 degrees
  ///////////////////////////////////*/

  //function for goal cover and move at same time
 thread t = thread([]
 {move(1916.82);});
 t.detach();
 goalCover.set(true);
 t.join();
  
 //normal functions
  frontClamp.set(true);
  wait(10, msec);
  move(-1791.81);
  Drivetrain.turnToHeading(250,degrees);
  goalCover.set(false);
  wait(10, msec);
  frontClamp.set(false);
  Drivetrain.driveFor(reverse, 12, inches, 75, velocityUnits::rpm);
  BackFortnite();
  FourBarConveyor.spin(reverse);
  Drivetrain.driveFor(forward, 20, inches, 150, velocityUnits::rpm);
  wait(0.5, seconds);
  BackFortnite2();


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

/*//////////////////////////////////////////////////////////////////////////

USERCONTROL (INCLUDING ALL FUNCTIONS)

//////////////////////////////////////////////////////////////////////////*/

void driveUsercontrol() {  // Tank drive
  while (true) {
    leftDrive.spin(forward, Controller1.Axis3.value(),  pct);
    rightDrive.spin(forward, Controller1.Axis2.value(), pct);
    
    
    
  }
}

// void fourBarUsercontrol() {  // FourBar - Axis 2 (Right Joystick)
//   if (Controller2.Axis2.value() > 2 || Controller2.Axis2.value() < -2) {
//     FourBar.spin(forward, -Controller2.Axis2.value(), pct);
//   } else {
//     FourBar.spin(forward, 0, pct);
//   }
// }

bool open=true;
void frontClamp1()
{
 open=!open;
}
 


void fourBarConveyorUserControl() {  // FourBar&Conveyor - Axis 2 (Right Joystick)
  if (Controller2.Axis2.value() > 2 || Controller2.Axis2.value() < -2) {
      FourBar.spin(forward, -Controller2.Axis2.value(), pct);
      FourBarConveyor.spin(forward, -Controller2.Axis2.value(), pct);
    } 
    
    else if (Controller2.ButtonA.pressing()) {
       arms.stop();
      FourBarLift.setVelocity(75.0, percent);
      FourBarLift.spin(reverse);
    } 
    else if (Controller1.ButtonR2.pressing()) {
       arms.stop();
      FourBarLift.setVelocity(75.0, percent);
      FourBarLift.spin(forward);
    } 
    else if (Controller1.ButtonDown.pressing()) {
        i = 1;
        arms.stop();
        FourBarLift.stop();
      Clamp.setVelocity(80.0, percent);
      Clamp.spin(forward);
    } 
    else if (Controller1.ButtonUp.pressing()) {
        i=0;
       arms.stop();
        FourBarLift.stop();
      Clamp.setVelocity(80.0, percent);
      Clamp.spin(reverse);
    } 
    else{
      arms.stop();
      FourBarLift.stop();
      if(i == 0){
      Clamp.stop();
      }
    }
    wait(1, msec);
  }
  return 0;


// void ConveyorBeltUserControl(){
//   if(Controller2.ButtonA.pressing()){
//     FourBarConveyor.spin(reverse);
//   }
// }

void frontClampUserControl(){
  if(Controller2.ButtonR1.pressing()){
    frontClamp.set(true);

  }
  else if(Controller2.ButtonR2.pressing()){
    frontClamp.set(false);
  }

}
void backClampUserControl(){

if(Controller2.ButtonL2.pressing()){
    backClamp1.set(true);
    backClamp2.set(true);
  }
  else if(Controller2.ButtonL1.pressing()){
    backClamp1.set(false);
    backClamp2.set(false);
  }
}

void usercontrol(void) {
  // Runs drive code as a thread so we can always control the robot
  thread(driveUsercontrol).detach();
  Drivetrain.setStopping(coast);
  
  while (true) {
    if(Controller2.ButtonR2.pressing()){
      vex::digital_out frontClamp(Brain.ThreeWirePort.A);
      frontClamp.set(true);
    }

    else if(Controller2.ButtonR1.pressing()){
      vex::digital_out frontClamp(Brain.ThreeWirePort.A);
      frontClamp.set(false);
    }
    //fourBarConveyorUserControl();  // Controls four bar up and down with contorller 2
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 2);
    Controller1.Screen.print(visionSample(NEUTRAL, 50));
    frontClampUserControl();
    backClampUserControl();
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
}
