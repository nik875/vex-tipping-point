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
}

/*//////////////////////////////////////////////////////////////////////////

AUTONOMOUS (INCLUDING ALL FUNCTIONS)

//////////////////////////////////////////////////////////////////////////*/

// OPTIMIZE
int timestep = 10;  // Delay for PID, in ms
// OPTIMIZE
double k = 10;  // Maximum amount drivetrain may increase/decrease power for PID
int maxVel = 200 - k;  // Maximum velocity that we're allowed to move
// OPTIMIZE
int ACC_RATE = 50;  // Maximum amount of acceleration (rpm / s)
int vel = 0;  // Velocity for PID (reset in move)
int targetPos = 0;  // Position we should be at in PID
// OPTIMIZE
int DEC_RATE = 50;  // Rate of decelleration with braking scheme (rpm / s)

double average(double a, double b) {  // Average of two numbers
  return (a + b) / 2;
}

double degreesToDistance(double deg) {  // Convert a degree value of an encoder to a distance
  return 0;
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

void move(double dist) {  // Drive forward a given distance with PID and acceleration curve
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
    double leftVel = vel + correct(leftLead);
    double rightVel = vel + correct(rightLead);
    targetPos += average(leftVel, rightVel) * timestep;
    leftDrive.spin(forward, leftVel, rpm);
    rightDrive.spin(forward, rightVel, rpm);
    wait(timestep, msec);
  }
  acc.interrupt();
  stop();
}

void autonomous(void) {

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
