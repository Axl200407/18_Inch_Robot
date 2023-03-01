#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
motor FL = motor(PORT11, ratio6_1, true);
motor FR = motor(PORT18, ratio6_1, false);
motor ML = motor(PORT12, ratio6_1, false);
motor MR = motor(PORT19, ratio6_1, true);
motor BL = motor(PORT13, ratio6_1, true);
motor BR = motor(PORT20, ratio6_1, false);
motor catapultLeft = motor(PORT1, ratio36_1,false);
motor catapultRight = motor(PORT10, ratio36_1,false);
motor_group catapult = motor_group(catapultLeft,catapultRight);
motor intake = motor(PORT2, ratio6_1, false);
motor rollers = motor(PORT9, ratio36_1, false);

controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT3);

limit cata_limit = limit(Brain.ThreeWirePort.H);


pneumatics expansion = pneumatics(Brain.ThreeWirePort.D);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}