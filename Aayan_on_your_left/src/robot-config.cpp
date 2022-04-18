#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1(primary);
controller Controller2(partner);
motor FourBar(PORT20, ratio36_1, true);
digital_out frontClamp(Brain.ThreeWirePort.E);
digital_out goalCover(Brain.ThreeWirePort.F);
digital_out backClamp1(Brain.ThreeWirePort.G);
digital_out backClamp2(Brain.ThreeWirePort.H);
motor LF(PORT18, ratio6_1, true);
motor LB(PORT5, ratio6_1, true);
motor RF(PORT10, ratio6_1, false);
motor RB(PORT11, ratio6_1, false);
motor LM(PORT8, ratio6_1, false);
motor RM(PORT4, ratio6_1, true); 
motor FourBarConveyor(PORT16, ratio6_1, false);
// vision AllSeeingEye(PORT11);
inertial Inertial(PORT9);
encoder leftEncoder(Brain.ThreeWirePort.A);
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