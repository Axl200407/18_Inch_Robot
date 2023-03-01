using namespace vex;

extern brain Brain;
extern motor FL;
extern motor FR;
extern motor ML;
extern motor MR;
extern motor BL;
extern motor BR;
extern motor catapultLeft;
extern motor catapultRight;
extern motor_group catapult;
extern motor intake;
extern motor rollers;

extern controller Controller1;
extern inertial Inertial;


extern limit cata_limit;

extern pneumatics expansion;
extern pneumatics CataPiston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
