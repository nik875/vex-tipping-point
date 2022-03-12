using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FourBar;
extern motor BackClamp;
extern motor Clamp;
extern motor Conveyor;
extern controller Controller1;
extern controller Controller2;
extern smartdrive Drivetrain;
extern bumper BackClampLimit;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );