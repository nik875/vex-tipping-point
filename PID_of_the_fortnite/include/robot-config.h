using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern controller Controller2;
extern motor FourBar;
extern motor FourBarConveyor;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor LM;
extern motor RM;
extern vision AllSeeingEye;
extern inertial Inertial;
extern encoder leftEncoder;
extern encoder rightEncoder;
extern motor_group leftDrive;
extern motor_group rightDrive;
extern smartdrive Drivetrain;
extern digital_out frontClamp;
extern digital_out goalCover;
extern digital_out backClamp1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );