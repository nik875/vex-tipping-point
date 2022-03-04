using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FourBar;
extern motor BackClamp;
extern motor Clamp;
extern motor Conveyor;
extern controller Controller1;
extern controller Controller2;
extern inertial Inertial;
extern motor LB;
extern motor LF;
extern motor RB;
extern motor RF;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );