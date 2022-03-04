#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FourBar = motor(PORT20, ratio36_1, false);
motor BackClamp = motor(PORT1, ratio18_1, false);
motor Clamp = motor(PORT21, ratio18_1, false);
motor Conveyor = motor(PORT16, ratio6_1, false);
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
inertial Inertial = inertial(PORT9);
motor LB = motor(PORT18, ratio18_1, false);
motor LF = motor(PORT5, ratio18_1, false);
motor RB = motor(PORT10, ratio18_1, true);
motor RF = motor(PORT2, ratio18_1, true);

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