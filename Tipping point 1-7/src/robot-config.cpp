#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LB = motor(PORT1, ratio18_1, false);
motor RB = motor(PORT2, ratio18_1, false);
motor LF = motor(PORT3, ratio18_1, false);
motor RF = motor(PORT4, ratio18_1, false);
motor ConveyerMotor = motor(PORT6, ratio18_1, false);
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
/*vex-vision-config:begin*/
signature Vision__SIG_1 = signature (1, 3967, 4577, 4272, -2375, -1955, -2165, 3, 0);
vision Vision = vision (PORT7, 50, Vision__SIG_1);
/*vex-vision-config:end*/
encoder EncoderA = encoder(Brain.ThreeWirePort.C);
motor Claw = motor(PORT5, ratio18_1, false);
motor Tilter = motor(PORT8, ratio18_1, false);
motor FourBar = motor(PORT9, ratio18_1, false);

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