#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors

controller Controller1(primary);
controller Controller2(partner);
motor FourBar(PORT1, ratio36_1, true);
motor FourBarConveyor(PORT6, ratio36_1, false);
digital_out frontClamp(Brain.ThreeWirePort.E);
digital_out goalCover(Brain.ThreeWirePort.F);
digital_out backClamp1(Brain.ThreeWirePort.G);
motor LF(PORT3, ratio6_1, true);
motor LB(PORT15, ratio6_1, true);
motor RF(PORT19, ratio6_1, false);
motor RB(PORT9, ratio6_1, false);
motor LM(PORT11, ratio6_1, false);
motor RM(PORT7, ratio6_1, true);
// vision AllSeeingEye(PORT11);
inertial Inertial(PORT4);
motor_group leftDrive(LF, LB, LM);
motor_group rightDrive(RF, RB, RM);
 smartdrive Drivetrain(leftDrive, rightDrive, Inertial, 3.25 * 3.14, 15.5, 10.5, inches, 1);
//smartdrive Drivetrain(leftDrive, rightDrive, Inertial, 299.24, 320, 40, inches, 1);
encoder leftEncoder = encoder(Brain.ThreeWirePort.A);
encoder rightEncoder = encoder(Brain.ThreeWirePort.C);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}