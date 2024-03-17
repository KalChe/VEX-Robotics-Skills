/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       cheru                                                     */
/*    Created:      3/16/2024, 9:32:08 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <vector>
#include <cmath>
#include <numeric>
#include <string>
#include <iostream>

using namespace std;
using namespace vex;

// A global instance of competition
competition Competition;
brain  Brain;
// define your global instances of motors and other devices here
motor RightDrive1 = motor(PORT10, ratio6_1, false);
motor RightDrive2 = motor(PORT9, ratio6_1, true);
motor RightDrive3 = motor(PORT8, ratio6_1, false);
motor LeftDrive1 = motor(PORT20, ratio6_1, true);
motor LeftDrive2 = motor(PORT19, ratio6_1, true);
motor LeftDrive3 = motor(PORT17, ratio6_1, false);
motor C2 = motor(PORT15, ratio36_1, true);
motor C1 = motor(PORT13, ratio36_1, false);
motor_group Cata = motor_group(C1, C2);
controller Controller1 = controller(primary);
motor Intake = motor(PORT5, ratio18_1, false);
digital_out Matic = digital_out(Brain.ThreeWirePort.A);
inertial Inert = inertial(PORT14);




void straight(double distance, int speed) {
LeftDrive1.spinFor(distance, deg, speed, velocityUnits::pct, false);
LeftDrive2.spinFor(distance, deg, speed, velocityUnits::pct, false);
LeftDrive3.spinFor(distance, deg, speed, velocityUnits::pct, false);
RightDrive1.spinFor(distance, deg, speed, velocityUnits::pct, false);
RightDrive2.spinFor(distance, deg, speed, velocityUnits::pct, false);
RightDrive3.spinFor(distance, deg, speed, velocityUnits::pct, true);
}

//motor_group LeftDrive = motor_group(LeftDrive1, LeftDrive2, LeftDrive3);
//motor_group RightDrive = motor_group(RightDrive1, RightDrive2, RightDrive3);
motor_group LeftDrive = motor_group(LeftDrive1, LeftDrive2, LeftDrive3);
motor_group RightDrive = motor_group(RightDrive1, RightDrive2, RightDrive3);

long double pi = atan(1)*4;

void cataDown(){
 Cata.spin(directionType::rev, 70, velocityUnits::pct);
   wait(5, seconds);
  Cata.stop(brakeType::hold); 

}

bool open = true;
void pneum(){
  Matic.set(open);
  open = !open;
}

void pneumopen(){
  Matic.set(true);
}

void pneumclose(){
  Matic.set(false);
}

void RightTurn(int turn){
  RightDrive.setVelocity(turn, percent);
  LeftDrive.setVelocity(turn, percent);
  RightDrive.spin(reverse);
  LeftDrive.spin(fwd);
}

void LeftTurn(int turn){
  RightDrive.setVelocity(turn, percent);
  LeftDrive.setVelocity(turn, percent);
  RightDrive.spin(fwd);
  LeftDrive.spin(reverse);
}

void stopMotors(vex::brakeType b){
  LeftDrive.stop(b);
  RightDrive.stop(b);
}

void RightAcc(double spinner){
  Inert.setRotation(0, degrees);

    int dt = 20;  // Recommended wait time in milliseconds
  double target = spinner; // In revolutions
  double error = target - Inert.rotation();
  double tP = .147; //.17
  double tD = 0.035; //.035
  double prevError = error;

  double timeElap = 0;

  while (timeElap < 1400) {
    error = target - Inert.rotation();
    double derivative = (error - prevError)/dt;
    double percent1 = tP * error + tD * derivative;

    Brain.Screen.printAt(20, 40, "Inertial: %f", Inert.rotation());
    Brain.Screen.printAt(20, 40, "Power: %f", Inert.rotation());

    RightTurn(percent1);

    vex::task::sleep(dt);
    prevError = error;

    timeElap += dt;
  }
  stopMotors(hold);
}

void LeftAcc(double spinner){
  Inert.setRotation(0, degrees);

  int dt = 20;  // Recommended wait time in milliseconds
  double target = spinner; // In revolutions
  double error = target - Inert.rotation();
  double tP = .147; //.17
  double tD = 0.035; //.035
  double prevError = error;

  double timeElap = 0;

  while (timeElap < 1400) {
    error = target - Inert.rotation();
    double derivative = (error - prevError)/dt;
    double percent2 = tP * error + tD * derivative;

    Brain.Screen.printAt(20, 40, "Inertial: %f", Inert.rotation());

    LeftTurn(percent2);

    vex::task::sleep(dt);
    prevError = error;

    timeElap += dt;
  }
  stopMotors(hold);
}


void pre_auton(void) {
  Inert.calibrate();
}


void autonomous(void) {
Inert.calibrate();
Cata.setVelocity(75, percent);
LeftDrive.setVelocity(50, percent);
RightDrive.setVelocity(50, percent);

straight(-220, 50);
LeftAcc(40);
straight(-2600, 60);
straight(500, 30);

RightAcc(180);
RightAcc(135);

straight(-1100, 30); // touches matchloading bar
RightAcc(85);
straight(-100, 40);
Cata.setVelocity(50, percent);
//Matchloading
Cata.spin(fwd);
wait(40, seconds); //30 matchloading
Cata.stop(hold);
straight(1000, 100);
RightAcc(90);
RightAcc(45);

straight(1200, 50);
LeftAcc(42.8);

double x = 1.8; // Change this value
Cata.setVelocity(10, percent);
while(Cata.torque(Nm) < x){
  Cata.spin(fwd); //Moves cata down enough to go under bar
}
  Cata.stop(hold);

straight(1000, 30);
straight(5750, 30); //touches opposite side matchloading bar and rotates 45 degrees
straight(-3500, 40); //moves straight touching middle bar
RightAcc(22.5); //aligns straight on with the goal
pneumopen(); //extends pneumatics

straight(-4000, 30);
straight(2000, 30);
straight(-2000, 30);
straight(2000, 30); //pushes in and out of goal twice

LeftAcc(45); //moves 45 degrees to the left
pneumclose();

straight(-2800, 30);

RightAcc(90);
RightAcc(45);

straight(1500, 40); //pushes in from the left side of the goal
}

bool shooting = false;
void cataShoot(){
Cata.setVelocity(50, percent);
while(true){
  while(Controller1.ButtonL1.pressing()){
    Cata.spin(fwd);
}
 if(Controller1.ButtonY.pressing()){
    shooting = !shooting;
    wait(0.1, seconds);
  }  
  if(shooting){
    Cata.spin(fwd);

  } else{
    Cata.stop(hold);}}}

int counter = 0;
void driveDirection(){
  counter++;
}

void drivercontrol() {
  thread CATA = thread(cataShoot); //Y sets cata shooting, and L1 held shoots
Controller1.ButtonX.pressed(pneum); //Opens/closes pneumatics
Controller1.ButtonA.pressed(driveDirection); //Changes which side of the drive is forward

 while (true) {

if(counter % 2 == 1){

LeftDrive.spin(fwd, Controller1.Axis3.position(), pct);
RightDrive.spin(fwd, Controller1.Axis2.position(), pct);

} else {

LeftDrive.spin(reverse, Controller1.Axis2.position(), pct);
RightDrive.spin(reverse, Controller1.Axis3.position(), pct);

}

if(Controller1.ButtonR1.pressing()){
 Intake.spin(reverse, 100, pct);
}
else if(Controller1.ButtonR2.pressing()){
 Intake.spin(fwd, 100, pct);
}
else{
Intake.stop(brakeType::hold);
wait(0.1, msec);
}
wait(5, msec);
}}

int main(){
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);
pre_auton();
wait(10, msec);
}