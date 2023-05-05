#ifndef __IMPLEMENTATION_H_
#define __IMPLEMENTATION_H_

#include "robot-config.h"
#include "tracking/robotModels.h"
#include "tracking/gui.h"

#define COMPETITION 1

#if COMPETITION
  competition Competition;
#endif

const double updateTime = 11; //msec
const double motionDelay = 12; //msec
const double cameraDelay = 20;

//Units/Rev, (in/rev)
//linGain, rotGain, maxV, maxW, maxAccel
//stopRadius, stopTime, errorRadius, errorTime, noNewTurnRadius, shiftRadius, cruiseVelocity, stopAngularRadius
//maxMoveToTurnAllowableAngleForMotion

robotProfile GusBotProfile = {
8.375 / 23.8264, {7,1}, {45,1}, 100, 200, 100,
0.01, 0.2, 1, 0.2, 8, 14, 70, degToRad(1.5), degToRad(45)
};

TankDrive robot(&leftDriveMotorGroup, &rightDriveMotorGroup, GusBotProfile);

const char fileName[] = "Hello.txt";
Log logger = Log(false);

Flywheel flywheel = Flywheel(&FlywheelGroup, 0, 0, 0);

int flywheelUpdate(){
  
  flywheel.clearSpeeds(0);
  flywheel.enable();
  while(true){
    flywheel.update();
  }
}

//TODO MEASUREE TORQUE
bool isJamming(){
  if( (fabs(frontIntake.torque()) > 0.2) && (fabs(frontIntake.velocity(percentUnits::pct)) < 2)  ){
    return true;
  }
  return false;
}

void fire(){
  //while(!flywheel.fireReady()){wait(15, msec);}
  trigger.rotateTo(95, rotationUnits::deg, 100, velocityUnits::pct);
  trigger.rotateTo(0, rotationUnits::deg, 100, velocityUnits::pct);
  wait(1100, msec);
}

const double UnitsPerRevLeft = 13.5 / 37.475;
const double UnitsPerRevRight = 13.5 / 38.2333;
const double UnitsPerRevHor = 16.0 / 45.7944;

double robotStartHeading = 0;
void track(){
  //Time Calculations
  static double lastTime = fabs(Brain.timer(timeUnits::msec) - 1);
  double deltaT = Brain.timer(timeUnits::msec) - lastTime;
  deltaT = deltaT / 1000;

  static double lastLeft = 0;
  static double lastRight = 0;

  double leftEnc = getLeftEnc();
  double rightEnc = getRightEnc();

  double dR = rightEnc-lastRight;
  double dL = leftEnc-lastLeft;
  double deltaFwd = 0.5*(dL*UnitsPerRevLeft + dR*UnitsPerRevRight);

  lastLeft = leftEnc;
  lastRight = rightEnc;

  //Heading Calculations
  static double lastHeading = robotStartHeading;
  double head = getHeadingUnbounded(); //Gets unbounded angle; Positive in CW direction
  double deltaHead = lastHeading - head; //+ is CCW; Heading given + in CW so sign is flopped
  lastHeading = head;
  head = 360 - normalizeAngle(head, false); //Heading in CCW + direction


  //deltaPos Calculations
  Vector2d deltaPos = Vector2d(deltaFwd, head, true);

  static double lastHor = 0;
  double horEnc = getHorEnc();
  double deltaHor = (horEnc-lastHor)*UnitsPerRevHor;
  lastHor = horEnc;

  deltaPos = deltaPos + Vector2d(deltaHor, head-90, true);

  //Update robot class
  //std::cout << deltaPos << ", " << deltaHead << std::endl;
  //positionSet p = robot.getCurrPos();
  //p.head = radToDeg(p.head);
  //std::cout << p << std::endl;
  //std::cout << robot.location.getCurrHead() << ", " << robot.location.getTargetHead() << std::endl;

  robot.shiftLocationGlobal(deltaPos, deltaHead, true);
  robot.setHead(head, true); //Set heading using more accurate inertial sensor

  robot.updateTracking(deltaT);
  robot.updateTargetVelocities(deltaT);

  robot.updateMotors();

  //Save last time
  lastTime = Brain.timer(timeUnits::msec);
}



//Thread for update cycle, also contains logging code for robot motion for later analysis with python tools
//Needs to be activated in preAuto function
int frame = 0;
int trakerFunction(){
  resetEncoders();
  Inertial.setHeading(360-robotStartHeading, rotationUnits::deg); //90 deg CCW but inertial sensor only measures in CW{% endif %}
  logger.save(fileName);

  while(true){
    track();

    logger.meta("FRAME", "START");
    logger.logTime(Brain.timer(timeUnits::msec));

    logger.logPointSet("cPoint", robot.getCurrPos());
    logger.logPointSet("tPoint", robot.getTargetPos());
    logger.logPointSet("nTPoint", robot.getNextTargetPos());

    logger.status("Rotate Only", robot.isRotateOnly());
    logger.logTimeout("L", robot.isDone(), robot.isMoving(), robot.isAtTarget(), robot.isLinTimedOut());
    logger.logTimeout("R", robot.isDone(), robot.isTurning(), robot.isAtRotTarget(), robot.isRotTimedOut());
    
    logger.status("LEFTM", leftDriveMotorGroup.velocity(percentUnits::pct));
    logger.status("RIGHTM", rightDriveMotorGroup.velocity(percentUnits::pct));
    
    //logger.print();

    #if COMPETITION
    if(Competition.isAutonomous()){
      logger.append(fileName);
    }
    #endif

    wait(updateTime, msec);
  }
}

//Update motor outputs based on robot class output
void executeMove(){
  bool inMotion = !robot.isDone();
  while(inMotion){
    wait(motionDelay, timeUnits::msec);
    inMotion = !robot.isDone();
  }
}

#endif