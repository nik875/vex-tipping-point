// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// leftEncoder          encoder       A, B            
// rightEncoder         encoder       C, D            
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
  frontClamp.set(true);
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
}

void BackFortnite2(){
  backClamp1.set(false);
  }



//PID FUNCTIONS


// OPTIMIZE
int timestep = 10;  // Delay for PID, in ms
// OPTIMIZE
double k = 20;  // Maximum amount drivetrain may increase/decrease power for PID
// OPTIMIZE
double kVis = 10;  // Maximum amount vision sensor may increase/decrease power
int maxVel = 200 - k;  // Maximum velocity that we're allowed to move
// OPTIMIZE
int ACC_RATE = 50;  // Maximum amount of acceleration (rpm / s)
int vel = 0;  // Velocity for PID (reset in move)
int targetPos = 0;  // Position we should be at in PID
// OPTIMIZE
int DEC_RATE = 50;  // Rate of decelleration with braking scheme (rpm / s)

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

void accelerate() {  // Increases vel as time passes
  while (vel + ACC_RATE / 100 <= maxVel) {
    vel += ACC_RATE / 100;
    wait(10, msec);
  }
  vel = maxVel;
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
    d += v * timestep;
    v -= DEC_RATE * timestep;
  }
  return d;
}

void stop() {  // Custom braking scheme
  while (average(leftEncoder.velocity(rpm), rightEncoder.velocity(rpm)) > 0) {
    Drivetrain.stop(brake);
    wait(10, msec);
    Drivetrain.stop(coast);
    wait(30, msec);
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
  // Reset encoder positions
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  vel = 0;  // Reset velocity
  targetPos = 0;  // Reset target position
  thread acc(accelerate);  // Update velocity independently of our code
  acc.detach();
  while (dist - degreesToDistance(  // While we have enough distance to stop
      average(
        leftEncoder.position(degrees), rightEncoder.position(degrees))
      ) > stopDistance()) {
    double leftLead = degreesToDistance(leftEncoder.position(degrees)) - targetPos;
    double rightLead = degreesToDistance(rightEncoder.position(degrees)) - targetPos;
    double leftVel = vel + correct(leftLead) + correctionFunc() / 2;  // Divide correctionFunc() by 2 to equally correct both sides
    double rightVel = vel + correct(rightLead) - correctionFunc() / 2;
    targetPos += average(leftVel, rightVel) * timestep;
    leftDrive.spin(forward, leftVel, rpm);
    rightDrive.spin(forward, rightVel, rpm);
    wait(timestep, msec);
  }
  acc.interrupt();
  stop();
}

void autonomous(void) {
  /*///////////////////////////////////
    1 DEGREE = 0.0239982 inches

    1 INCH = 41.6697919 degrees
  ///////////////////////////////////*/

  
 move(10000);
 


  
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

int x=0, y=1;

int ConveyorValue=0;

bool open=true;
void frontClamp1()
{
 open=!open;
}
 


// void fourBarConveyorUserControl() {  // FourBar&Conveyor - Axis 2 (Right Joystick)
//   if (Controller2.Axis2.value() > 0) {
//     FourBar.spin(forward, -Controller2.Axis2.value(), pct);
//     FourBarConveyor.spin(forward, -Controller2.Axis2.value(), pct);
//   } else if(Controller2.Axis2.value() < 0) {
//     FourBar.spin(forward, 0, pct);
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

void ConveyorUserControl(){
  if(Controller2.ButtonA.pressing()){
    ConveyorValue=1;
  }
  if(Controller2.ButtonX.pressing()){
    ConveyorValue=0;
  }
  if(abs(Controller2.Axis2.value()) < 2){
    if(ConveyorValue==1){
      FourBarConveyor.spin(reverse);
    }
    else if(ConveyorValue==0){
      FourBarConveyor.stop();
    }
  }
}

void fourBarUsercontrol(){
if (Controller2.Axis2.value() > 0) {
    FourBar.spin(forward, Controller2.Axis2.value(), pct);
    FourBarConveyor.spin(forward, Controller2.Axis2.value(), pct);
  } else if(Controller2.Axis2.value() < 0) {
    FourBar.spin(forward, 0, pct);
  }
else{
  FourBar.spin(forward, 0, pct);
  FourBarConveyor.spin(forward, 0, pct);
}
}






void backClampUserControl(){

if(Controller2.ButtonL2.pressing()){
   vex::digital_out backClamp1(Brain.ThreeWirePort.G);
    backClamp1.set(true);
  }
  else if(Controller2.ButtonL1.pressing()){
     vex::digital_out backClamp1(Brain.ThreeWirePort.G);
    backClamp1.set(false);
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
    if(Controller2.ButtonL1.pressing()){
      vex::digital_out backClamp1(Brain.ThreeWirePort.G);
      backClamp1.set(true);

    }

    else if(Controller2.ButtonR1.pressing()){
      vex::digital_out backClamp1(Brain.ThreeWirePort.G);
      backClamp1.set(false);
    }
    
    fourBarUsercontrol();  // Controls four bar up and down with contorller 2
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 2);
    Controller1.Screen.print(visionSample(NEUTRAL, 50));
    // backClampUserControl();
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
