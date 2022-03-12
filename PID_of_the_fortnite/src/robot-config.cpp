#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1(primary);
controller Controller2(partner);
motor FourBar(PORT20, ratio36_1, false);
motor Conveyor(PORT16, ratio6_1, false);
bumper BackClampLimit(Brain.ThreeWirePort.A);
motor LF(PORT18, ratio18_1, false);
motor LB(PORT5, ratio18_1, false);
motor RF(PORT10, ratio18_1, true);
motor RB(PORT2, ratio18_1, true);
motor LM(PORT1, ratio18_1, false);
motor RM(PORT4, ratio18_1, true);
inertial Inertial(PORT9);
encoder leftEncoder(Brain.ThreeWirePort.B);
encoder rightEncoder(Brain.ThreeWirePort.C);
motor_group leftDrive(LF, LB, LM);
motor_group rightDrive(RF, RB, RM);
smartdrive Drivetrain(leftDrive, rightDrive, Inertial, 4 * 3.14, 15.5, 10.5, inches, 1);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}