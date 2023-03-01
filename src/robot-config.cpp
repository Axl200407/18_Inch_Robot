#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
motor FL = motor(PORT11, ratio18_1, true);
motor FR = motor(PORT18, ratio18_1, false);
motor ML = motor(PORT12, ratio18_1, false);
motor MR = motor(PORT19, ratio18_1, true);
motor BL = motor(PORT13, ratio18_1, true);
motor BR = motor(PORT20, ratio18_1, false);
motor catapult = motor(PORT10, ratio18_1,false);
motor intake = motor(PORT1, ratio18_1, false);
motor rollers = motor(PORT5, ratio6_1, false);

controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT3);

limit cata_limit = limit(Brain.ThreeWirePort.A);


pneumatics Endgame = pneumatics(Brain.ThreeWirePort.D);
pneumatics CataPiston = pneumatics(Brain.ThreeWirePort.C);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}