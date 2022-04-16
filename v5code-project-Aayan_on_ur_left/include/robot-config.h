using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern controller Controller2;
extern motor FourBar;
extern motor Conveyor;
extern motor LF;
extern motor LB;
extern motor RF;
extern motor RB;
extern motor LM;
extern motor RF;
extern vision AllSeeingEye;
extern inertial Inertial;
extern encoder leftEncoder;
extern encoder rightEncoder;
extern motor_group leftDrive;
extern motor_group rightDrive;
extern smartdrive Drivetrain;
extern pneumatics frontClamp;
extern pneumatics goalCover;
extern pneumatics backClamp1;
extern pneumatics backClamp2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );