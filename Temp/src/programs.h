#ifndef __PROGRAMS_H_
#define __PROGRAMS_H_
#include "implementation.h"

//START PROGRAMS-----------------------------------------------------------


// TODO Implement program editor here
// TODO Make a basic program before connecting this all to the program to ensure the startHeading and all those changes work

//END PROGRAMS-------------------------------------------------------------

/*
EXAMPLE:

void rightSpecial(int mod){
  Do Auto stuff
}
autoEntry entries[] = {
  {"Right Special", rightSpecial},
};
*/


void a(int mod){
  //Get Rollers
  robot.activateCV(Vector2d(0, -7), 0);
  Roller1.spin(forward, 60, percentUnits::pct);
  while(leftDriveMotorGroup.torque() < 0.4){wait(10, msec);}
  robot.activateCV(Vector2d(0, 0), 0);
  Roller1.spinFor(50, rotationUnits::deg, 60, velocityUnits::pct);
  robot.activateCV(Vector2d(0, 14), 0);
  wait(500, msec);
  robot.activateCV(Vector2d(0, 0), 0);
  FlywheelGroup.spin(fwd, 88, pct);
  double timeA = Brain.timer(timeUnits::msec);
  robot.activatePID();
  robot.setAbsTarget(0, 2);
  executeMove();
// Get 1st discs
  robot.turnTo(128, true);
  frontIntake.spin(forward, 100, pct);
  executeMove();
  robot.setAbsTarget(Vector2d(6.2, 137, true));
  executeMove();
  robot.forward = false;

  robot.turnTo(130, true);
  robot.setAbsTarget(Vector2d(-3.7, 137, true));
  executeMove();

  robot.forward = true;
  robot.turnTo(101, true);// FIRST SHOT
  executeMove();
  while(Brain.timer(timeUnits::msec) - timeA < 4000){wait(10, msec);}
  for(int i =0; i<4; i++){fire();}

  robot.forward = true;
  robot.turnTo(39, true);
  executeMove();
  robot.maxV = 70;
  robot.setAbsTarget(Vector2d(15, 39, true));
  executeMove();


  robot.maxV = 21;
  robot.setAbsTarget(Vector2d(27, 39, true));
  double lastJamActive = 0;
  while(!robot.isDone()) {
    if(  (isJamming()) && (Brain.timer(timeUnits::msec) - lastJamActive > 50)  ){
      frontIntake.spin(forward, -100, pct);
      wait(200, msec);
      frontIntake.spin(forward, 100, pct);
      lastJamActive = Brain.timer(timeUnits::msec);
    }
    wait(15, msec);
  }


  
  //executeMove();

  FlywheelGroup.spin(fwd, 85, pct);
  timeA = Brain.timer(timeUnits::msec);
  robot.turnTo(120, true);//2ND SHOT
  executeMove();
  while(Brain.timer(timeUnits::msec) - timeA < 1000){wait(10, msec);}
  for(int i =0; i<4; i++){fire();}


  robot.setAbsTarget(15, 5.75);
  executeMove();  
  robot.turnTo(290, true);
  executeMove();

  robot.stopMotors();
  robot.activateCV(Vector2d(0, 40), 0);
  positionSet c = robot.getCurrPos();
  while((c.p.y > 15) && (c.head > M_PI) && (c.head < degToRad(340))){c = robot.getCurrPos();wait(10, msec);}
  if((c.head > M_PI) || (c.head < degToRad(340))){
    robot.activateCV(Vector2d(0, -20), 0);
    logger.warning("Auton", "Jammed on wall");
    wait(500, msec);
  }
  robot.activateCV(Vector2d(0, 0), 0);

  robot.activatePID();
  robot.maxV = 100;
  robot.turnTo(290, true);
  executeMove();
  robot.setAbsTarget(robot.getCurrPos().p);
  robot.forward = false;
  robot.setAbsTarget(-6, 12);
  executeMove();
  robot.forward = true;
  
  FlywheelGroup.spin(fwd, 86, pct);
  timeA = Brain.timer(timeUnits::msec);
  robot.turnTo(132, true);//3RD SHOT
  executeMove();
  while(Brain.timer(timeUnits::msec) - timeA < 200){wait(10, msec);}
  for(int i =0; i<4; i++){fire();}
}

void skills(int mod) {
  a(0);
  robot.turnTo(190, true);
  executeMove();
  robot.maxV = 90;
  robot.setAbsTarget(Point2d(-16, 10));
  executeMove();
  robot.turnTo(130, true);
  executeMove();
  robot.setAbsTarget(Vector2d(10, 130, true));
  executeMove();
  robot.turnTo(0, true);
  executeMove();

  robot.activateCV(Vector2d(0, -7), 0);
  Roller1.spin(forward, 60, percentUnits::pct);
  while(leftDriveMotorGroup.torque() < 0.4){wait(10, msec);}
  robot.activateCV(Vector2d(0, 0), 0);
  Roller1.spinFor(50, rotationUnits::deg, 60, velocityUnits::pct);
  robot.activateCV(Vector2d(0, 14), 0);
  wait(500, msec);
  robot.activateCV(Vector2d(0, 0), 0);

  robot.activatePID();
  robot.turnTo(-45, true);
  executeMove();
  robot.setRelTarget(0, 4.5);
  executeMove();
  robot.turnTo(45, true);
  executeMove();

  //fire endgame
  endGamePne.set(true);
}


//Storage of program functions for use in the GUI
typedef struct {
  std::string name;
  void (*function)(int);
  int mod;
  startingPosition startPos;
} autoEntry;

//Use of a table allows programs to be easily added or removed
autoEntry entries[] = {
  {"Main", a, 0, {Point2d(0, 0), 90, true}},
  {"Skills", skills, 0, {Point2d(0, 0), 90, true}},
};
int indexAuto = 0;


//START OF GUI------------------------------------------------------------------------
bool programRunning = false;

//Implementation of a button to increase the auto index to change which function is selected
void increaseIndex(){
  indexAuto += 1;
  if(indexAuto > (sizeof(entries)/sizeof(entries[0]))-1){
    indexAuto = 0;
  }
}

void drawAAA(int x, int y, int w, int h, bool press){
  if(press){
    Brain.Screen.setFillColor(green);
  }else{
    Brain.Screen.setFillColor(red);
  }
  Brain.Screen.drawRectangle(x, y, w, h);
  Brain.Screen.setFillColor(white);
  Brain.Screen.drawLine((w/4)+x, (3*h/4)+y, (w/2)+x, (h/4)+y);
  Brain.Screen.drawLine((3*w/4)+x, (3*h/4)+y, (w/2)+x, (h/4)+y);
}
ButtonGUI increaseBTN(480-50,0,50,70, increaseIndex, drawAAA);

void decreaseIndex(){
  indexAuto -= 1;
  if(indexAuto < 0){
    indexAuto = (sizeof(entries)/sizeof(entries[0]))-1;
  }
}
void drawAAB(int x, int y, int w, int h, bool press){
  if(press){
    Brain.Screen.setFillColor(green);
  }else{
    Brain.Screen.setFillColor(red);
  }
  Brain.Screen.drawRectangle(x, y, w, h);
  Brain.Screen.setFillColor(white);
  Brain.Screen.drawLine((w/4)+x, (h/4)+y, (w/2)+x, (3*h/4)+y);
  Brain.Screen.drawLine((3*w/4)+x, (h/4)+y, (w/2)+x, (3*h/4)+y);
}
ButtonGUI decreaseBTN(480-50,80,50,70, decreaseIndex, drawAAB);

bool GUIActivate = true;
void setAuto(){
  GUIActivate = false;
}
void drawAAC(int x, int y, int w, int h, bool press){
  if(press){
    Brain.Screen.setFillColor(white);
  }else{
    Brain.Screen.setFillColor(red);
  }
  Brain.Screen.drawRectangle(x, y, w, h);
  if(press){
    Brain.Screen.setFillColor(green);
  }else{
    Brain.Screen.setFillColor(white);
  }
  Brain.Screen.drawCircle((w/2)+x, (h/2)+y, (w/4));
}
ButtonGUI setBTN(480-50,160,50,70, setAuto, drawAAC);

//All buttons used in GUI
ButtonGUI buttons[] = {increaseBTN, decreaseBTN, setBTN};

//Calls a button if pressed
void pressHandler(){
  if(!GUIActivate){return;}
  static bool free = true;

  if(free){
    free = false;
  }else{
    return;
  }

  if(Brain.Screen.pressing()){
    for(int i=(sizeof(buttons)/sizeof(ButtonGUI))-1; i >= 0; i--){
      if(buttons[i].call(Brain.Screen.xPosition(), Brain.Screen.yPosition())){
        buttons[i].draw();
        waitUntil(!Brain.Screen.pressing());
        buttons[i].isPressed = false;
        free = true;
        return;
      }
    }
  }
}

//Main GUI loop that draws/animates button objects
void BrainGUIProgram(){
  Brain.Screen.pressed(pressHandler);

  while(GUIActivate){
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    std::string out = entries[indexAuto].name;
    Brain.Screen.print(out.c_str());

    for(int i=0; i < (sizeof(buttons)/sizeof(ButtonGUI)); i++){
      buttons[i].draw();
    }
    Brain.Screen.render();
    wait(50, msec);
  }

  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  std::string out = entries[indexAuto].name;
  Brain.Screen.print("Selected: %s", out.c_str());
  drawLoadingScreen();
  Brain.Screen.render();
}

//Auto function used by competition template
void autonomous(void){
  GUIActivate = false;

  double ang = entries[indexAuto].startPos.currentHeading;
  if(!entries[indexAuto].startPos.currentHeadingInDeg){
    ang = radToDeg(ang);
  }
  ang = normalizeAngle(ang, false);
  robotStartHeading = ang;
  robot.setStartingPos(entries[indexAuto].startPos.currentPosition, ang, true);

  task traker = task(trakerFunction);
  wait(updateTime+1, msec);

  robot.activatePID();
  entries[indexAuto].function(entries[indexAuto].mod);
}
#endif