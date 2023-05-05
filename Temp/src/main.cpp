/*---------------------------------------------------------------------------

    Module:       main.cpp
    Author:       AUBIE1
    Created:      Thu Sep 26 2019
    Description:  AUBIE1 Generated Code

-----------------------------------------------------------------------------*/


#include "programs.h"
#include "driver.h"

void pre_auton(void) {
  robot.stopMotors();
  setupMotors();

  drawLoadingScreen();
  
  logger.info("Inertial", "Calibrating");
  Inertial.startCalibration();
  //trigger.spin(fwd, -100, pct);
  wait(250, msec);
  while(Inertial.isCalibrating()){wait(20, msec);}
  wait(250, msec);
  logger.info("Inertial", "Done");

/*
  while(trigger.torque() < 0.5 && abs(trigger.velocity(percentUnits::pct)) > 1) {
    wait(20, msec);
  }*/
  trigger.resetPosition();
  trigger.stop();

  //std::cout << "DONE CALIBRATING" << std::endl;

  Brain.Screen.clearScreen();
  #if COMPETITION
    BrainGUIProgram();
  #else
    robotStartHeading = 90;
    task traker = task(trakerFunction);
    wait(updateTime+1, msec);
  #endif
  //task flywheelThread = task(flywheelUpdate);
}


//Main functon
int main() {
  pre_auton();

  #if COMPETITION
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
  #endif

  //flywheel.enable();
  //flywheel.setTarget(80);
  //frontIntake.spin(fwd, 100, pct);
  while(true) {
    //std::cout << getLeftEnc() << ", " << getRightEnc() << ", " << getHorEnc() << std::endl;
    //std::cout << robot.getCurrPos() << std::endl;;

    //std::cout << frontIntake.torque() << ", " << frontIntake.velocity(percentUnits::pct) << std::endl;;;;;;;
    wait(2000, msec);
  }
}

