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
motor LF = motor(PORT3, ratio6_1, true);
motor LB = motor(PORT15, ratio6_1, true);
motor RF = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT9, ratio6_1, false);
motor LM = motor(PORT11, ratio6_1, false);
motor RM = motor(PORT7, ratio6_1, true);
motor_group leftDrive = motor_group(LF, LB, LM);
motor_group rightDrive = motor_group(RF, RB, RM);
// vision AllSeeingEye(PORT11);
inertial Inertial(PORT4);
smartdrive Drivetrain(leftDrive, rightDrive, Inertial, 3.25 * 3.14, 15.5, 10.5, inches, 1);
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