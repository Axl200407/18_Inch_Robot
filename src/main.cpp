/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ---

#include "vex.h"

using namespace vex;




competition Competition;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

bool toggleR = true;
bool toggleL = true;

#define FL_TURN_CONST 0.012//0.0149364441
#define FR_TURN_CONST -0.012//-0.0149364441
#define ML_TURN_CONST 0.012//0.0149364441
#define MR_TURN_CONST -0.012//-0.0149364441
#define BL_TURN_CONST 0.012//0.0149364441
#define BR_TURN_CONST -0.012//-0.0149364441

// Acceleration Parameter
#define ACC_PARAM 2

float integral = 0;
// Define parameter turnSpeed which represents the real time maximum as
// calculated by the acceleration format
float turnSpeed = 0;
int facing = 0;

void turnTo(float desired_heading, unsigned int speed) {
    // This function will determine the correct direction to turn, and the speed
    // to turn at unsigned integers are better because they take less
    // computational power on the brain

    // FORMAT OF CALL : turnTo(desired_heading, speed)
    // INPUTS         : desired_heading -> the desired global heading to which the
    //                  robot should turn in degrees on the range [0,360]
    //                  speed -> the turning speed as a positive % value
    // OUTPUTS        : void
    // EFFECTS        : robot will turn to specified heading at the specified
    //                speed
    //

    // determine the necesary change in heading (dH)
    // the d repreents delta, or change, while H represents heading.
    // dH -> change in heading
    float dH = desired_heading - (180 - Inertial.heading(degrees));


    facing = desired_heading;

    float maxSpeed = 0;
    double trueMaxSpeed = 0;
    turnSpeed = 0;
    int counter = 0;

    integral = 0;

    while ((fabs(dH) >= .75) || (fabs(FR.velocity(pct)) >= 1 && counter != 1)) {
      counter = 1;
        // determine if the magnitude of dH is greater than 180
        dH = desired_heading - (180 - Inertial.heading(degrees));
        while (fabs(dH) > 180) {
            // if so, determine i++f positive or negative
            if (dH > 0) {
                // if positive, subtract 360
                dH -= 360;
            }

            else {
                // else, add 360
                dH += 360;
            }
        }
        dH = dH * -1;


        // The current way of deceleration is inherently flawed, I'll comment it out for now.

        // 0.010 is 10ms delay
        /*float DECCEL_dH = turnSpeed * (turnSpeed / (ACC_PARAM / 0.010)) -
                      1 / 2 *
                          (ACC_PARAM *
                            switch to multiplication if not working  0.010) *
                          pow(turnSpeed / (ACC_PARAM / 0.010), 2);*/

        // replacement: 
        float decel_limit = (pow(turnSpeed * 495 / 100, 2) * 0.020 / 2 / ACC_PARAM / 60 / 495) *10;

        // dH now represents the sign and magnitude of the fastest turn. Pos
        // values indicate a counterclockwise turn, while negative values
        // indicate a clockwise turn

        // using trig, geometry, and algebra, we can write an equation to
        // represent the linear relationship between desired turning angle
        // in degrees and wheel rotations.
        float FL_speed = FL_TURN_CONST * dH;
        float FR_speed = FR_TURN_CONST * dH;
        float ML_speed = ML_TURN_CONST * dH;
        float MR_speed = MR_TURN_CONST * dH;
        float BL_speed = BL_TURN_CONST * dH;
        float BR_speed = BR_TURN_CONST * dH;

        // This is how our maximum wheel speed will change when the robot
        // is accelerating

        // Minimum distance from target that will allow the robot to deccelerate
        // at the desired ACC_PARAM

        float speedArray[6] = { FL_speed, FR_speed, BL_speed, BR_speed, ML_speed, MR_speed };
        for (int i = 0; i < 6; i++) {
            if (fabs(speedArray[i]) > maxSpeed) {
                maxSpeed = fabs(speedArray[i]);
            }
        }


        

        if (maxSpeed < fabs(decel_limit)) {
            turnSpeed -= ACC_PARAM;
        }
        else {
            // if turnSpeed >= speed, turnSpeed = speed
            // else turnSpeed += acceleration param
            turnSpeed = turnSpeed >= speed ? speed : turnSpeed + ACC_PARAM;
        }


        integral += 0.01;


        FL.spin(forward, (FL_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);
        FR.spin(forward, (FR_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);
        ML.spin(forward, (ML_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);
        MR.spin(forward, (MR_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);
        BL.spin(forward, (BL_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);
        BR.spin(forward, (BR_speed / maxSpeed * turnSpeed) + integral, velocityUnits::pct);

        // Brain.Screen.print(FL_speed);
        // or maybe just spin(M_speed) then stop after loop?

        // wait to avoid brain overheat
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Inertial: ");
        Brain.Screen.setCursor(1, 11);
        Brain.Screen.print(Inertial.heading(degrees));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("dH: ");
        Brain.Screen.setCursor(2, 4);
        Brain.Screen.print(dH);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("maxSpeed: ");
        Brain.Screen.setCursor(3, 9);
        Brain.Screen.print(maxSpeed);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("(fabs(maxSpeed) >= fabs(turnCheck)");
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print(trueMaxSpeed);
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print(FR.velocity(pct));

        Brain.Screen.setCursor(1, 20);
        Brain.Screen.print("decel_limit: ");
        Brain.Screen.setCursor(1, 34);
        Brain.Screen.print(decel_limit);
        Brain.Screen.setCursor(2, 20);
        Brain.Screen.print("integral: ");
        Brain.Screen.setCursor(1, 32);
        Brain.Screen.print(integral);
      
        wait(20, msec);
        Brain.Screen.clearScreen();
    }

    FL.stop(hold);
    FR.stop(hold);
    ML.stop(hold);
    MR.stop(hold);
    BL.stop(hold);
    BR.stop(hold);
}

// float kP = .3;//.2
// float kI = 1;
// float kD = 2.5;
// float kT = 0;
float kP = .05;//.2
float kI = .23;
float kD = 15;
float kT = 0;

float Proportion;
float Derivative;
float Integral;
float Turn;

float motor_speed = 0;

float prevError;
float IntegralBounds = 360;

float error_angle;
float turn_correct;

void drive(float desired, float speed){
  //Desired inputed in inches
  float error = 0;

  prevError = 0;
  turn_correct = 0;
  Integral = 0;
  Derivative = 0;


  desired = desired*(360/(1.7*M_PI));

  error = desired - FL.position(degrees) + FR.position(degrees)/(-2);

  FL.setPosition(0, degrees);
  FR.setPosition(0, degrees);
  ML.setPosition(0, degrees);
  MR.setPosition(0, degrees);
  BL.setPosition(0, degrees);
  BR.setPosition(0, degrees);
  int counter = 0;
  while ((fabs(error) >= 20) || (fabs(FR.velocity(pct)) >= 1 && counter != 1)) {
    counter = 1;

    error = desired - FL.position(degrees) + FR.position(degrees)/(-2);

    Proportion = error;

    prevError = error;
    Derivative =  error - prevError;

    if(fabs(error) <= IntegralBounds){
      Integral += 0.01;
    }else{
      Integral = 0;
    }

    

    float motor_speed =  (Proportion*kP) + (Integral*kI) + (Derivative*kD);

    float left_speed = 0;
    float right_speed = 0;


    if(facing == 0 && Inertial.heading(degrees) > 300){
      error_angle =  fabs(facing - Inertial.heading(degrees)) - 360;
    }else if(facing == 0 && Inertial.heading(degrees) < 45){
      error_angle = Inertial.heading(degrees);
    }
    else{
      error_angle = facing - Inertial.heading(degrees);
    }

    
    if (error_angle > 0){
      right_speed = motor_speed - fabs(error_angle * kT);
      left_speed = motor_speed;
    }else if(error_angle < 0){
      left_speed = motor_speed - fabs(error_angle * kT);
      right_speed = motor_speed;
    }else{
      right_speed = motor_speed;
      left_speed = motor_speed;
    }

    FL.spin(forward, (left_speed/100) * speed, velocityUnits::pct);
    FR.spin(forward, (right_speed/100) * speed, velocityUnits::pct);
    ML.spin(forward, (left_speed/100) * speed, velocityUnits::pct);
    MR.spin(forward, (right_speed/100) * speed, velocityUnits::pct);
    BL.spin(forward, (left_speed/100) * speed, velocityUnits::pct);
    BR.spin(forward, (right_speed/100) * speed, velocityUnits::pct);

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("motor_speed: ");
    Brain.Screen.setCursor(1, 13);
    Brain.Screen.print(motor_speed);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("error: ");
    Brain.Screen.setCursor(2, 7);
    Brain.Screen.print(error);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("errorAngle: ");
    Brain.Screen.setCursor(3, 13);
    Brain.Screen.print(error_angle);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("turn speed: ");
    Brain.Screen.setCursor(4, 15);
    Brain.Screen.print((Turn * kT));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Integrak: ");
    Brain.Screen.setCursor(5, 15);
    Brain.Screen.print((Integral));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("left speed: ");
    Brain.Screen.setCursor(7, 12);
    Brain.Screen.print((left_speed/100) * speed);

    wait(20,msec);
    Brain.Screen.clearScreen();
  }
  FL.stop(hold);
  FR.stop(hold);
  ML.stop(hold);
  MR.stop(hold);
  BL.stop(hold);
  BR.stop(hold);
}

bool toggle = true;
bool toggle1 = true;
// void PistonToggle1(){
//   if (toggle){
// 	  Endgame.open();
// 	  toggle = false;
//   }else if (toggle == false){
// 	  Endgame.close();
// 	  toggle = true;
//   }
// }
// void PistonToggle2(){
//   if (toggle1){
// 	  CataPiston.open();
// 	  toggle1 = false;
//   }else if (toggle1 == false){
// 	  CataPiston.close();
// 	  toggle1 = true;
//   }
// }
bool armset = true;
void retractCatapult(){
  armset=false;
  if(!armset && !cata_limit.pressing()){
    catapultLeft.spin(fwd, 100, pct);
  }
  else{
    catapultLeft.stop(hold);
    armset=true;
  }
}

void launchCatapult(){
  while(cata_limit.pressing()){
    catapultLeft.spin(fwd, 100, pct);
  }
  //catapult.stop(hold);
  wait(500,msec);
  retractCatapult();
}

void expansionButton(){
  expansion.close();
  //wait(250, msec);
  //expansion.open();
}

// void reload(){
//   while(!cata_limit.pressing()){
//       catapult.spin(forward, 100, pct);
//     }
// }

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(Inertial.isCalibrating()){
    wait(20,msec);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating");
  }
  Brain.Screen.clearScreen();
  //retractCatapult();
}


float inch =12/4;

void autonomous(void) {
  Inertial.setHeading(180, deg);
  drive(29.75, 30);
  turnTo(42, 15);
  wait(1000, msec);
  drive(-34.2, 40);
  turnTo(0, 20);
  wait(1000, msec);
  drive(-3.75,60);
  rollers.spin(fwd, 500, pct);
  wait(150, msec);
  rollers.stop(coast);
  drive(2.75,60);
  turnTo(42, 15);//15
  drive(67, 30);
  turnTo(-43, 15);//15
  wait(500, msec);
  drive(-6, 60);
  while (!cata_limit.pressing()){
    catapult.spin(fwd, 100, pct);
  }
  catapult.stop(hold);
  wait(500, msec);
  catapult.spin(fwd, 50, pct);
  wait(500, msec);
  catapult.stop(hold);
  while (!cata_limit.pressing()){
    catapult.spin(fwd, 100, pct);
  }
  catapult.stop(hold);
  intake.spin(fwd, 100, pct);
  wait(1500, msec);
  intake.stop(coast);
  catapult.spin(fwd, 50, pct);
  wait(500, msec);
  catapult.stop(hold);
} 

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  while (true){
    // if(Controller1.Axis1.value()>20){
      FL.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()*0.5), pct);
      ML.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()*0.5), pct);
      BL.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()*0.5), pct);
      FR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()*0.5), pct);
      MR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()*0.5), pct);
      BR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()*0.5), pct);
    // }else{
    //   FL.spin(fwd, Controller1.Axis3.value(), pct);
    //   ML.spin(fwd, Controller1.Axis3.value(), pct);
    //   BL.spin(fwd, Controller1.Axis3.value(), pct);
    //   FR.spin(fwd, Controller1.Axis3.value(), pct);
    //   MR.spin(fwd, Controller1.Axis3.value(), pct);
    //   BR.spin(fwd, Controller1.Axis3.value(), pct);
    // }
    

    //catapult control
    if (Controller1.ButtonL1.pressing()){
      catapult.spin(fwd, 100, pct);
    } else if (!cata_limit.pressing()){
      catapult.spin(fwd, 50, pct);
    } else {
      catapult.stop(hold);
    }
    //Controller1.ButtonL2.pressed(retractCatapult);
    //Controller1.ButtonL1.pressed(launchCatapult);

    //Spin intake. Right Bumper
    if (Controller1.ButtonR1.pressing()){
      intake.spin(fwd, 100, pct);
    } else if(Controller1.ButtonR2.pressing()){
      intake.spin(fwd, -100, pct);
    }
    else{
      intake.stop(hold);
    }


    //Spin roller. x and B.
    if (Controller1.ButtonX.pressing()){
      rollers.spin(fwd, 100, pct);
    } else if (Controller1.ButtonB.pressing()){
      rollers.spin(fwd, -100, pct);
    } else {
      rollers.stop(hold);
    }

    //expansion
    Controller1.ButtonDown.pressed(expansionButton);

    //wait(5,msec);
  }
}

int main() {
  //Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    /*
    if (Controller1.ButtonB.pressing()) {
      drive(3, 75);
      wait(250,msec);
      Piston.open();
      drive(-6, 50);
      turnTo(315,20);
      wait(250,msec);
      drive(65, 50);
      wait(250,msec);
      turnTo(45,25);
      wait(250,msec);
      catapult.spin(forward, 100, pct);
      wait(250,msec);
      while(!cata_limit.pressing()){
        catapult.spin(forward, 100, pct);
      }
      catapult.stop(hold);

      FL.stop(coast);
      ML.stop(coast);
      BL.stop(coast);
      FR.stop(coast);
      MR.stop(coast);
      BR.stop(coast);
    }
    FL.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()), pct);
    ML.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()), pct);
    BL.spin(fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()), pct);
    FR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()),pct);
    MR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()),pct);
    BR.spin(fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()), pct);

    
    //if (Controller1.ButtonL2.pressing()){
    //  catapult.spin(fwd, 100, pct);
    //}else{
    //  catapult.stop(hold);
    //}
    //if(!cata_limit.pressing()){
    //  reload();
    //}  

    if (Controller1.ButtonR2.pressing()){
      intake.spin(fwd, -100, pct);
    }else if (Controller1.ButtonR1.pressing()){
      intake.spin(fwd, 100, pct);
    }else{
      intake.stop(hold);
    }


    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(Inertial.heading(degrees)); */
    wait(10, msec);
    //Brain.Screen.clearScreen();
  }
}