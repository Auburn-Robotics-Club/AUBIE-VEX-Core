#ifndef __ROBOTMODELS_H__
#define __ROBOTMODELS_H__

/*
Name: robotModels.h
Written By: Carson Easterling

TODO

!!!!!Keep Linear PC but make rotation PID

Smooth path transitions between points when setting veloctity
build in support releying current vel for accel if jumping becoems an issue
*/

#include "robotmath.h"
#include "logger.h"
#include "navigation.h"

#include "v5.h"
#include "v5_vcs.h"

typedef struct{
  Point2d currentPosition;
  double currentHeading;
  bool currentHeadingInDeg;
}startingPosition;

typedef struct {
  double p;
  double i;
} MotionPI;

typedef struct{
  double UnitsPerRev;

  MotionPI linGain;
  MotionPI rotGain;
  double maxV;
  double maxW;
  double maxAccel;

  double stopRadius;
  double stopTime;
  double errorRadius;
  double errorTime;
  double noNewTurnRadius;
  double shiftRadius;
  double cruiseVelocity;

  double stopAngularRadius;

  double maxMoveToTurnAllowableAngleForMotion;
} robotProfile;

class Robot : public Navigator{
protected:
  Vector2d targetVelocity = Vector2d(0, 0); //Magnitude is the Pct velocity, and theta is direction of travel; Realitive to the current direction of the robot
  double targetAngularVel = 0; //Pct 

  Vector2d outputVelocity = Vector2d(0, 0);
  double outputAngularVelocity = 0;

  int mode = 0;
  Vector2d userDesiredVel = Vector2d(0, 0);
  double userDesiredSpeed = 0;
  double userDesiredAngularSpeed = 0;

public:
  MotionPI linearGains = {0, 0}; //Pct per Unit Error
  MotionPI rotGains = {0, 0};

  double maxV; //Pct
  double maxW;
  double maxAccel; //Pct/S
  double cruiseVelocity;

  double maxMoveToTurnAllowableAngleForMotion;


  bool setMotorControls = false;

  Robot(robotProfile profile): Navigator(profile.stopRadius, profile.stopTime, profile.errorRadius, profile.errorTime, profile.noNewTurnRadius, profile.shiftRadius, profile.stopAngularRadius){ 
    linearGains = profile.linGain;
    rotGains = profile.rotGain;
    maxV = profile.maxV;
    maxW = profile.maxW;
    maxAccel = profile.maxAccel;

    maxMoveToTurnAllowableAngleForMotion = profile.maxMoveToTurnAllowableAngleForMotion;
    //UnitsPerRev = profile.UnitsPerRev;
    cruiseVelocity = profile.cruiseVelocity;
  }

  void updateTargetVelocities(double deltaT){
    //TODO build in support for current vel for accel

    Vector2d error = getLocalError();
    Vector2d rError = getReverseLocalError();
    Vector2d nError = getNextLocalError();
    double hError = getHeadError();

    static Vector2d oldTargetVel = Vector2d(0, 0);
    oldTargetVel = targetVelocity;

    static double oldTargetW = 0;
    oldTargetW = targetAngularVel;

    if(isNextTargetLast() && mode==2){
      mode = 0;
    }

    //Set Target Velocites
    if(!setMotorControls){
      resetHeadingDerivitive();
      resetHeadingIntergral();
    }

    if(mode == 0){
      //PID

      //Tangential PID
      if(headingIndependence){
        //Heading Independent
        
        //Magnitude of PID(e) but direction of error
        double mag = 0;
        if(error.getMagnitude() > errorRadius){
          mag = error.getMagnitude()*linearGains.p + linearGains.i;
          /*
          if(error.getMagnitude() < rError.getMagnitude()){
            mag = error.getMagnitude()*linearGains.p + linearGains.i;
          }else{
            mag = rError.getMagnitude()*reverseLinearGains.p + reverseLinearGains.i;
          }*/
        }
        targetVelocity = Vector2d(mag, getTranslationalLocalHeading(), false);
      }else{
        //Not heading independant

        //Magnitude of PID(e) but only on y axis
        if(fabs(error.getY()) > errorRadius){
          targetVelocity = Vector2d(0, error.getY()*linearGains.p + linearGains.i*sign(error.getY()));
          /*
          if(abs(error.getY()) < abs(rError.getY())){
            targetVelocity = Vector(0, error.getY()*linearGains.p + linearGains.i*sign(error.getY()));
          }else{
            targetVelocity = Vector(0, (abs(rError.getY())*reverseLinearGains.p + reverseLinearGains.i)*sign(error.getY()));
          }*/
        }else{
          targetVelocity = Vector2d(0, 0);
        }
      }
    }else if (mode == 1) {
      //Pure Constant velocity
      targetVelocity = userDesiredVel;
      targetAngularVel = userDesiredAngularSpeed;

      resetHeadingDerivitive();
      resetHeadingIntergral();
    }else if(mode == 2){
      if(headingIndependence){
        targetVelocity = Vector2d(cruiseVelocity, getTranslationalLocalHeading(), false);
      }else{
        //Hvae proportional to the length of remaining arclength
        targetVelocity = Vector2d(0, cruiseVelocity); //Needs to take into account headingError alongside error and next error angle
      }

    }else if(mode == 3){
      //CV with targets

      if(error.getMagnitude() > errorRadius){
        targetVelocity = error.getUnitVector().scale(userDesiredSpeed);
      }else{
        targetVelocity = Vector2d(0, 0);
      }

      if(fabs(hError) > stopAngularRadius){
        targetAngularVel = sign(hError)*userDesiredAngularSpeed;
      }else{
        targetAngularVel = 0;
      }

      resetHeadingDerivitive();
      resetHeadingIntergral();
    }

    //Dont allow any edit of angular speed during CV function
    if(mode != 1){
      if(!headingIndependence){
        if(fabs(hError) > stopAngularRadius){
          targetAngularVel = hError*rotGains.p + rotGains.i*sign(hError);
        }else{
          targetAngularVel = 0;
        }
      }else{
        //If mode 3 CV with target function sets headingIndepence true then it has an angular magnitude it wants to turn at so this should be ignored
        if(mode != 3){
          if(fabs(hError) > stopAngularRadius){
            targetAngularVel = hError*rotGains.p + rotGains.i*sign(hError); //hError*rotGains.p + getHeadingIntergral()*rotGains.i + getHeadingDerivitive()*rotGains.d; Maybe get rid of the 0 thing and use full PID?
          }else{
            targetAngularVel = 0;
          }
        }
      }
    }

    //After target vel is set its then check for acceleration
    if(targetVelocity.getMagnitude() > maxV){
      targetVelocity = targetVelocity.getUnitVector().scale(maxV);
    }
    Vector2d dV = targetVelocity - oldTargetVel;
    if(dV.getMagnitude()*sign(dV.dot(oldTargetVel))/deltaT > maxAccel){
      targetVelocity = oldTargetVel + dV.getUnitVector().scale(maxAccel*deltaT);
    }

    if(!headingIndependence){
      double angleAdjustSpeed = 0;
      if(mode == 0){
        //Standard PID
        angleAdjustSpeed = fabs(hError) < maxMoveToTurnAllowableAngleForMotion ? 1 : 0;
        targetVelocity.scale(angleAdjustSpeed);
      }else if(mode == 2){
        //Curve PID
        angleAdjustSpeed = fmax(0, 1 - 32.828*fabs(hError)*fabs(hError)); // TODO Test (+-10  degrees results in  as parabolic)
        //angleAdjustSpeed = 1 - (2*abs(hError)/PI)*(errorRadius/error.getMagnitude()); // TODO TEST
        targetVelocity.scale(angleAdjustSpeed);
      }
    }

    outputVelocity = targetVelocity;


    if(fabs(targetAngularVel) > maxW){
      targetAngularVel = sign(targetAngularVel)*maxW;
    }
    outputAngularVelocity = targetAngularVel; 
  }


  //Outputs are local to the robot
  Vector2d getTangentVelocity(){
    return outputVelocity;
  }

  double getAngularVelocity(){
    return outputAngularVelocity;
  }

  void activatePID(){
    setMotorControls = true;
    mode = 0;
  }
  void activatePath(){
    setMotorControls = true;
    mode = 2;
  }

  void activateCV(Vector2d cv, double w){
    setMotorControls = true;
    userDesiredVel = cv;
    userDesiredAngularSpeed = w;
    rotateTargetSetOnly = false;
    mode = 1;
  }

  void activateSpeedTarget(double speed, double angularSpeed){
    headingIndependence = true;
    setMotorControls = true;
    userDesiredSpeed = speed;
    userDesiredAngularSpeed = angularSpeed;
    rotateTargetSetOnly = false;
    mode = 3;
  }

  void stopMotors(){
    setMotorControls = false;
  }
  bool isDone(){
    //std::cout << isMoving() << ", " << isTurning() << ", " << !isAtTarget() << ", " << !isAtRotTarget() << ", " << isTimedOut() << std::endl;
    //std::cout << radToDeg(getHeadError()) << ", " << radToDeg(currentPos.head) << std::endl;
    //std::cout << "L: " << isMoving() << ", " << isAtTarget() << ", " << isLinTimedOut() << std::endl;
    //std::cout << "R: " << isTurning() << ", " << isAtRotTarget() << ", " << isRotTimedOut() << std::endl;

    if(mode == 0){
      if(headingIndependence){
        // Check to see if Lin or Rot should timeout the function depending on what mode is ongoing or if CV functions are in use ignore timeouts
        if(isRotateOnly()){
          if(isRotTimedOut()){
            return true;
          }
        }else{
          if(isRotTimedOut() && isLinTimedOut()){
            return true;
          }
        }
      }else{
        if(isLinTimedOut()){
          return true;
        }
      }
    }

    if(isRotateOnly()){
      if(isTurning() || !isAtRotTarget()){
        return false;
      }else{
        return true;
      }
    } else {
      if(isMoving() || isTurning() || !isAtTarget() || !isAtRotTarget()){
        return false;
      }else{
        return true;
      }
    }
  }
};

void setM(vex::motor m, double speed, vex::velocityUnits uni=vex::velocityUnits::pct){
  m.setVelocity(speed, uni);
  if(speed == 0){
    m.stop();
  }else{
    m.spin(vex::forward);
  }
}

void setSide(vex::motor_group m, double speed, vex::velocityUnits uni=vex::velocityUnits::pct){
  m.setVelocity(speed, uni);
  if(speed == 0){
    m.stop();
  }else{
    m.spin(vex::forward);
  }
}

class TankDrive : public Robot{
  protected:
    vex::motor_group* leftSide;
    vex::motor_group* rightSide;
  
  public:
    TankDrive(vex::motor_group* leftSideArg, vex::motor_group* rightSideArg,
              robotProfile profile) : Robot(profile){
      leftSide = leftSideArg;
      rightSide = rightSideArg;
    }

    void updateMotors(){
      double speed = getTangentVelocity().getY();
      double omega = getAngularVelocity();

      double left = -omega;
      double right = omega;

      if(!isRotateOnly()){
        left += speed;
        right += speed;
      }

      if(fabs(right) > maxV){
        left = left - (fabs(right) - maxV)*sign(left);
        right = maxV*sign(right);
      }
      if(fabs(left) > maxV){
        right = right - (fabs(left) - maxV)*sign(left);
        left = maxV*sign(left);
      }

      if(setMotorControls){
        setSide(*leftSide, left);
        setSide(*rightSide, right);
      }
    }
};

class XDrive : public Robot{
  protected:
    vex::motor_group* NWMotor;
    vex::motor_group* NEMotor;
    vex::motor_group* SWMotor;
    vex::motor_group* SEMotor;
  
  public:
    XDrive(vex::motor_group* NWMotorArg, vex::motor_group* NEMotorArg, vex::motor_group* SWMotorArg, vex::motor_group* SEMotorArg, 
           robotProfile profile) : Robot(profile){
      NEMotor = NEMotorArg;
      SEMotor = SEMotorArg;
      NWMotor = NWMotorArg;
      SWMotor = SWMotorArg;
    }

    void updateMotors(){
      Vector2d vel = getTangentVelocity();
      double speed = vel.getMagnitude();
      double travelAngle = Vector2d(1, 0).getAngle(vel);
      double omega = 0.25*getAngularVelocity();

      //std::cout << vel << std::endl;

      //std::cout << getCurrPos().p.x << ", " << getCurrPos().p.y << ", " << radToDeg(getCurrPos().head) << std::endl;
      //std::cout << vel << " / " << omega << " / " << getLocalError() << " / " << radToDeg(getHeadError()) << std::endl;

      double NESW_Coefficent = (sin(travelAngle) - cos(travelAngle)) / (2*sqrt(2));
      double NWSE_Coefficent = (sin(travelAngle) + cos(travelAngle)) / (2*sqrt(2));
      
      //-, +, -, + to go CCW
      //NW, NE, SW, SE
      double speeds[4] = {NWSE_Coefficent*speed - omega,  //NW
                          NESW_Coefficent*speed + omega,  //NE
                          NESW_Coefficent*speed - omega,  //SW
                          NWSE_Coefficent*speed + omega}; //SE

      int iMax = -1;
      double maxDiff = 0;
      for(int i=0; i<4; i++){
        double diff = fabs(speeds[i]) - 100;
        if(diff > 0){
          if(diff > maxDiff){
            iMax = i;
            maxDiff = diff;
          }
        }
      }

      if(iMax > -1){
        double sol = sign(speeds[iMax]) * 100;
        if(iMax == 0){
          speed = (sol + omega) / NWSE_Coefficent;
        }else if(iMax == 1){
          speed = (sol - omega) / NESW_Coefficent;
        }else if(iMax == 2){
          speed = (sol + omega) / NESW_Coefficent;
        }else{
          speed = (sol - omega) / NWSE_Coefficent;
        }
      }
      


      if(setMotorControls){
        //std::cout << speed << ", " << omega << std::endl;

        setSide(*NEMotor, NESW_Coefficent*speed + omega);
        setSide(*NWMotor, NWSE_Coefficent*speed - omega);
        setSide(*SEMotor, NWSE_Coefficent*speed + omega);
        setSide(*SWMotor, NESW_Coefficent*speed - omega);
      }
    }
};
#endif