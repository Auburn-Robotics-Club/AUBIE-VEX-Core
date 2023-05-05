#ifndef __LOCATOR_H__
#define __LOCATOR_H__

/*
Name: navigation.h
Written By: Carson Easterling

TODO

Simplify the multiple points navigation
Allow targetPosition or target HEading to be changed without impacting reverse PID or other PID factors


Figure out while face() doesn't work

Object tracking - Distance to another object on the field that can change base on sensor data
Waypoints
Pre-planned paths
Bezier paths or path will go through a set of defined points
Event Poitns where a call back will occur upon arriving or crossing at a point
*/

#include "robotmath.h"
#include "logger.h"

bool operator==(const positionSet& a, const positionSet& b){
  if((fabs(a.p.x - b.p.x) < 0.0001) && (fabs(a.p.y - b.p.y) < 0.0001) && (fabs(a.head - b.head) < 0.0001)){
    return true;
  }else{
    return false;
  }
}

bool operator!=(const positionSet& a, const positionSet& b){
  return !(a == b);
}

class Navigator{
protected:
  positionSet currentPos = {Point2d(0, 0), 0};
  positionSet previousPos = {Point2d(0, 0), 0};
  positionSet lastStoppedPos = {Point2d(0, 0), 0};

  Vector2d tangentialVelocity = Vector2d(0, 0); //units per second
  double angularVelocity = 0; //radians per second

  Vector2d tangentialAcceleration = Vector2d(0, 0); //units per second per second
  double angularAcceleration = 0; //radians per second per second

  
  positionSet currentTarget = {Point2d(0, 0), 0};
  positionSet nextTarget = {Point2d(0, 0), 0};
  positionSet lastTarget = {Point2d(0, 0), 0};

  int targetPathIndex = 0;
  std::vector<positionSet> targetPath;

  bool headingIndependence = false; //Determines if the heading is controlled by the tangential error direction
  bool atTarget = true;
  bool isStopped = true;
  bool atRotTarget = true;
  bool isStoppedRot = true;
  bool linTimedOut = false;
  bool rotTimedOut = false;
  double linearTimeoutTimer = 0;
  double rotTimeoutTimer = 0;
  bool rotateTargetSetOnly = false;

  double intergratedRotationErrorRespectToTime = 0;
  double derivitiveOfRotationErrorRespectToTime = 0;
  double lastHeadingError = 0;

  void setCurrentTargetTargetHeading(double x){
    currentTarget.head = x;
    resetHeadingDerivitive();
    resetHeadingIntergral();
    resetRotTimeout();
  }

public:
  double stopRadius;
  double stopTime;
  double errorRadius;
  double errorTime;
  double noNewTurnRadius;
  double shiftRadius;
  double stopAngularRadius;
  double timeoutTime = 1.25; //Seconds
  bool forward = true;

  Navigator(double stopRadiusArg, double stopTimeArg, double errorRadiusArg, double errorTimeArg, double noNewTurnRadiusArg, 
            double shiftRadiusArg, double stopAngularRadiusArg){
    currentPos = {Point2d(0, 0), M_PI_2};
    currentTarget = currentPos;
    lastTarget = currentPos;
    nextTarget = currentPos;
    previousPos = currentPos;
    lastStoppedPos = currentPos;

    stopRadius = stopRadiusArg;
    stopTime = stopTimeArg;
    errorRadius = errorRadiusArg;
    errorTime = errorTimeArg;
    noNewTurnRadius = noNewTurnRadiusArg;
    shiftRadius = shiftRadiusArg;
    stopAngularRadius = stopAngularRadiusArg;
  }

  void setStartingPos(Point2d currentPosition, double currentHeading, bool currentHeadingInDeg){
    if(currentHeadingInDeg){
      currentHeading = degToRad(currentHeading);
    }
    currentHeading = normalizeAngle(currentHeading);

    currentPos = {currentPosition, currentHeading};
    setSinglePointPath(currentPos, false);

    //std::cout << currentTarget << std::endl;

    previousPos = currentPos;
    lastStoppedPos = currentPos;

    atRotTarget = true;
    atTarget = true;
  }

  void setHead(double newHead, bool headingInDegrees){
    if(headingInDegrees){
      newHead = degToRad(newHead);
    }
    newHead = normalizeAngle(newHead);
    currentPos.head = newHead;
  }
  void setCurrentPosition(Point2d p){
    currentPos.p = p;
  }
  void setCurrentPosition(double x, double y){
    currentPos.p = Point2d(x, y);
  }

  //Updates Position data based on global transformations
  void shiftLocationGlobal(Vector2d deltaPos, double deltaHeading, bool headingInDegrees=false){
    if(headingInDegrees){
      deltaHeading = degToRad(deltaHeading);
    }

    currentPos.p = deltaPos + currentPos.p;
    currentPos.head = normalizeAngle(currentPos.head + deltaHeading);

    //std::cout << currentPos.p << std::endl;
  }

  //Updates Position data based on local transformations
  void shiftLocationLocal(Vector2d deltaFwd, double deltaHeading, bool headingInDegrees=false){
    Vector2d deltaPos = Vector2d(deltaFwd.getY(), currentPos.head) + Vector2d(deltaFwd.getX(), currentPos.head - M_PI_2);
    shiftLocationGlobal(deltaPos, deltaHeading, headingInDegrees);
  }

  //Updates the targeting system and error vectors
  void updateTracking(double deltaT){
    // Update PID Tracking Vars
    double headError = getHeadError();
    intergratedRotationErrorRespectToTime += headError*deltaT;
    derivitiveOfRotationErrorRespectToTime = (headError - lastHeadingError) / deltaT;
    lastHeadingError = getHeadError();

    //Stop Timers
    static double stopTimer = 0;
    static double errorTimer = 0;
    static double rotStopTimer = 0;
    static double rotErrorTimer = 0;

    //Motion stop detection
    Vector2d motion = Vector2d(previousPos.p, currentPos.p);
    Vector2d newV = motion.scale(1/deltaT);
    tangentialAcceleration = (newV - tangentialVelocity).scale(1/deltaT);
    tangentialVelocity = newV;

    //std::cout << motion.getMagnitude() << ", " << stopRadius << ", " << stopTimer << ", " << stopTime << ", " << deltaT << std::endl;
    if(motion.getMagnitude() < stopRadius){
      stopTimer += deltaT;
      if(stopTimer > stopTime){
        lastStoppedPos.p = currentPos.p;
        isStopped = true;
      }
    }else{
      stopTimer = 0;
      isStopped = false;
    }

    //Rotation stopped detection
    double angularDifference = shortestArcToTarget(previousPos.head, currentPos.head);
    double newOmega = angularDifference / deltaT;
    angularAcceleration = (newOmega - angularVelocity)/deltaT;
    angularVelocity = newOmega;

    //std::cout << angularDifference << std::endl;

    if(fabs(angularDifference) < stopAngularRadius){
      rotStopTimer += deltaT;
      if(rotStopTimer > stopTime){
        lastStoppedPos.head = currentPos.head;
        isStoppedRot = true;
      }
    }else{
      rotStopTimer = 0;
      isStoppedRot = false;
    }

    //Target changes
    Vector2d error = Vector2d(currentPos.p, currentTarget.p);
    if(targetPath.size() > 1){
      while((error.getMagnitude() < shiftRadius) && (targetPathIndex < targetPath.size() - 1)){
        targetPathIndex++;
        lastTarget = currentTarget;
        currentTarget = targetPath[targetPathIndex];

        resetLinTimeout();

        if(targetPathIndex < targetPath.size() - 1){
          nextTarget = targetPath[targetPathIndex+1];
        }else{
          nextTarget = currentTarget;
        }

        error = Vector2d(currentPos.p, currentTarget.p);
      }
    }

    //Update headingTarget if in dependence mode
    if(!headingIndependence){
      if(targetPathIndex < targetPath.size() - 1){
        double a = error.getMagnitude();
        double b = getNextGlobalError().getMagnitude();
        double c = Vector2d(currentPos.p, nextTarget.p).getMagnitude();

        double q = (a*a + b*b - c*c)/(2*a*b);
        if(q != 1){
          double R = c / (2*sqrt(1-q*q));
          double shiftTheta = 0.5*acos((a*a-2*R*R)/(-2*R*R));
          shiftTheta = fabs(shiftTheta) * -1 * sign(error.getAngle(getNextGlobalError()));

          setCurrentTargetTargetHeading(Vector2d(1, 0).getAngle(error) + shiftTheta);
        }else{
          setCurrentTargetTargetHeading(Vector2d(1, 0).getAngle(error));
        }
      }else{
        if(error.getMagnitude() > noNewTurnRadius){
          double targetHead = Vector2d(1, 0).getAngle(error);
          if(!forward){
            targetHead = targetHead + M_PI;
            targetHead = normalizeAngle(targetHead);
          }
          setCurrentTargetTargetHeading(targetHead);
        }
      }
    }

    //Error within bounds detection

    //Motion arrived detection
    if( ( (headingIndependence) && (error.getMagnitude() < errorRadius) ) || ( (!headingIndependence) && (getLocalError().getY() < errorRadius) ) ){
      errorTimer += deltaT;
      if(errorTimer > errorTime){
        atTarget = true;
      }
    }else{
      errorTimer = 0;
      atTarget = false;
    }


    //Motion arrived detection
    double angularError = fabs(shortestArcToTarget(currentPos.head, currentTarget.head));
    //std::cout << radToDeg(currentTarget.head) << ", " << atRotTarget << ", " << radToDeg(currentPos.head) << std::endl;
    //std::cout << radToDeg(angularError) << std::endl;
    if(angularError < stopAngularRadius){
      rotErrorTimer += deltaT;
      if(rotErrorTimer > errorTime){
        atRotTarget = true;
      }
    }else{
      rotErrorTimer = 0;
      atRotTarget = false;
    }
    
    if(!isMoving() && !isAtTarget()){
      linearTimeoutTimer += deltaT;
      if(linearTimeoutTimer > timeoutTime){
        linTimedOut = true;
      }
    }else{
      linearTimeoutTimer = 0;
      linTimedOut = false;
    }

    if(!isTurning() && !atRotTarget){
      rotTimeoutTimer += deltaT;
      if(rotTimeoutTimer > timeoutTime){
        rotTimedOut = true;
      }
    }else{
      rotTimeoutTimer = 0;
      rotTimedOut = false;
    }

    previousPos = currentPos;
  }

  //Targeting setters
  void setDir(bool fwd=true){
    forward = fwd;
  }

  void setSinglePointPath(positionSet T, bool rotateOnly=false){
     //TODO Figure out how to set Head and target position without having to update all this; Ideally I can set position then change the head target based on camera input
    lastTarget = currentTarget;
    currentTarget = T;
    nextTarget = T;

    targetPath.clear();
    targetPath.push_back(T);
    targetPathIndex = 0;

    atRotTarget = false;
    atTarget = false;
    rotateTargetSetOnly = rotateOnly;
    resetHeadingDerivitive();
    resetHeadingIntergral();
    resetLinTimeout();
    resetRotTimeout();
  }

  void setMultiPointPath(std::vector<positionSet> &newList){
    int size = newList.size();
    if(size > 0){
      lastTarget = currentTarget;
      currentTarget = newList.at(0);

      if(size > 1){
        nextTarget = newList.at(1);
      }else{
        nextTarget = newList.at(0);
      }

      targetPath.clear();

      targetPath = newList;

      targetPathIndex = 0;

      forward = true;
      atRotTarget = false;
      atTarget = false;
      rotateTargetSetOnly = false;
      resetHeadingDerivitive();
      resetHeadingIntergral();
      
      resetLinTimeout();
      resetRotTimeout();
    }
  }

  void setAbsTarget(double deltaX, double deltaY){
    setAbsTarget(Vector2d(deltaX, deltaY));
  }
  void setAbsTarget(Vector2d v){
    setAbsTarget(v + currentTarget.p);
  }
  void setAbsTarget(Point2d p){
    double targetHead = Vector2d(1, 0).getAngle(Vector2d(currentPos.p, p));
    if(!forward){
      targetHead = targetHead + M_PI;
      targetHead = normalizeAngle(targetHead);
    }

    headingIndependence = false;

    setSinglePointPath({p, targetHead});
  }


  void setAbsTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false){
    setAbsTarget(Vector2d(deltaX, deltaY), targetHead, inDeg);
  }
  void setAbsTarget(Vector2d v, double targetHead, bool inDeg=false){
    setAbsTarget(v + currentTarget.p, targetHead, inDeg);
  }

  void setAbsTarget(Point2d p, double targetHead, bool inDeg=false){
    headingIndependence = true;

    if(inDeg){
      targetHead = degToRad(targetHead);
    }
    targetHead = normalizeAngle(targetHead);

    setSinglePointPath({p, targetHead});
  }

  void setRelTarget(double deltaX, double deltaY){
    setRelTarget(Vector2d(deltaX, deltaY));
  }
  void setRelTarget(Vector2d v){
    Vector2d r = Vector2d(v.getY(), currentTarget.head, false);
    r = r + Vector2d(v.getX(), currentTarget.head - M_PI_2, false);
    setAbsTarget(r);
  }
  void setRelTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false){
    setRelTarget(Vector2d(deltaX, deltaY), targetHead, inDeg);
  }
  void setRelTarget(Vector2d v, double targetHead, bool inDeg=false){
    Vector2d r = Vector2d(v.getY(), currentTarget.head, false);
    r = r + Vector2d(v.getX(), currentTarget.head - M_PI_2, false);
    setAbsTarget(r, targetHead, inDeg);
  }

  void turnTo(double head, bool inDeg=false){
    headingIndependence = true;
    
    if(inDeg){
      head = degToRad(head);
    }
    head = normalizeAngle(head);

    setSinglePointPath({currentTarget.p, head}, true);
  }
  void turn(double deltaHeading, bool inDeg=false){
    if(inDeg){
      deltaHeading = degToRad(deltaHeading);
    }
    turnTo(currentTarget.head + deltaHeading);
  }
  void face(Point2d p){
    turnTo(Vector2d(1, 0).getAngle(Vector2d(currentPos.p, p)));
  }
  void face(Vector2d v){
    turnTo(Vector2d(1, 0).getAngle(v));
  }

  Vector2d translateGlobalToLocal(Vector2d v){
    return v.getRotatedVector(M_PI_2 - currentPos.head);
  }
  Vector2d translateLocalToGlobal(Vector2d v){
    return v.getRotatedVector(currentPos.head - M_PI_2);
  }

  Vector2d getRobotNormalVector(){
    return Vector2d(1, currentPos.head, false);
  }
  Vector2d getLocalError(){
    return translateGlobalToLocal(getGlobalError());
  }
  Vector2d getReverseLocalError(){
    return translateGlobalToLocal(getReverseGlobalError());
  }
  Vector2d getNextLocalError(){
    return translateGlobalToLocal(getNextGlobalError());
  }
  Vector2d getGlobalError(){
    return Vector2d(currentPos.p, currentTarget.p);
  }
  Vector2d getReverseGlobalError(){
    return Vector2d(lastStoppedPos.p, currentTarget.p);
  }
  Vector2d getNextGlobalError(){
    return Vector2d(currentTarget.p, nextTarget.p);
  }
  bool isNextTargetLast(){
    return !(targetPathIndex < targetPath.size() - 1);
  }
  double getHeadError(){
    return shortestArcToTarget(currentPos.head, currentTarget.head);
  }
  double getReverseHeadError(){
    //TODO DOESNT WORK
    return shortestArcToTarget(lastStoppedPos.head, currentPos.head);
  }
  double getTranslationalGlobalHeading(){
    return Vector2d(1, 0).getAngle(getGlobalError());
  }
  double getTranslationalLocalHeading(){
    return Vector2d(1, 0).getAngle(getLocalError());
  }
  Vector2d getGlobalVelocity(){
    return tangentialVelocity;
  }
  Vector2d getLocalVelocity(){
    return translateGlobalToLocal(tangentialVelocity);
  }
  Vector2d getGlobalAcceleration(){
    return tangentialAcceleration;
  }
  Vector2d getLocalAcceleration(){
    return translateGlobalToLocal(tangentialAcceleration);
  }
  double getAngularVelocity(){
    return angularVelocity;
  }

  positionSet getCurrPos(){
    return currentPos;
  }
  positionSet getTargetPos(){
    return currentTarget;
  }
  positionSet getNextTargetPos(){
    return nextTarget;
  }

  bool isMoving(){
    return !isStopped;
  }
  bool isTurning(){
    return !isStoppedRot;
  }
  bool isAtTarget(){
    return atTarget;
  }
  bool isAtRotTarget(){
    return atRotTarget;
  }
  bool isLinTimedOut(){
    return linTimedOut;
  }
  bool isRotTimedOut(){
    return rotTimedOut;
  }
  bool isRotateOnly(){
    return rotateTargetSetOnly;
  }

  void resetHeadingIntergral(){
    intergratedRotationErrorRespectToTime = 0;
  }
  double getHeadingIntergral(){
    return intergratedRotationErrorRespectToTime;
  }

  void resetHeadingDerivitive(){
    lastHeadingError = getHeadError();
    derivitiveOfRotationErrorRespectToTime = 0;
  }
  double getHeadingDerivitive(){
    return derivitiveOfRotationErrorRespectToTime;
  }

  void resetRotTimeout(){
    rotTimeoutTimer = 0;
    rotTimedOut = false;
  }

  void resetLinTimeout(){
    linearTimeoutTimer = 0;
    linTimedOut = false;
  }
};

#endif