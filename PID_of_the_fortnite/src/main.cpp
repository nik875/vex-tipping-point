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

// OPTIMIZE
int timestep = 10;  // Delay for PID, in ms
// OPTIMIZE
double k = 10;  // Maximum amount drivetrain may increase/decrease power for PID
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
    wait(timestep, msec);
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

bool canStop(double dist) {  // If we have enough distance left to stop our robot
  return dist - degreesToDistance(
    average(
      leftEncoder.position(degrees), 
      rightEncoder.position(degrees))) 
    > stopDistance();
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
  while (canStop(dist)) {  // While we have enough space left to stop our robot
    double leftLead = targetPos - degreesToDistance(leftEncoder.position(degrees));
    double rightLead = targetPos - degreesToDistance(rightEncoder.position(degrees));
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
  thread([]{move(12, alignNeutral);}).detach();  // Move 12 inches asynchronously while aligning to neutral goal
  // move(12, alignNeutral);
}

/*//////////////////////////////////////////////////////////////////////////

USERCONTROL (INCLUDING ALL FUNCTIONS)

//////////////////////////////////////////////////////////////////////////*/

void driveUsercontrol() {  // Tank drive
  while (true) {
    leftDrive.spin(forward, Controller1.Axis3.value(), pct);
    rightDrive.spin(forward, Controller1.Axis2.value(), pct);
  }
}

void fourBarUsercontrol() {  // FourBar - Axis 2 (Right Joystick)
  if (Controller2.Axis2.value() > 2 || Controller2.Axis2.value() < -2) {
    FourBar.spin(forward, -Controller2.Axis2.value(), pct);
  } else {
    FourBar.spin(forward, 0, pct);
  }
}

void usercontrol(void) {
  // Runs drive code as a thread so we can always control the robot
  thread(driveUsercontrol).detach();
  while (true) {
    fourBarUsercontrol();  // Controls four bar up and down with contorller 2
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 2);
    Controller1.Screen.print(visionSample(NEUTRAL, 50));
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
