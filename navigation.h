#ifndef ROBOT_NAV_H
#define ROBOT_NAV_H

#include "robotmath/robotmath.h"
#include "logger.h"
#include "v5.h"
#include "v5_vcs.h"

// Look into these functions found in v5_api.h
  //vexDeviceGetByIndex(0);
  //vexSerialReadChar(9600);

//TrackingBase
//--------------------------------------------------------------------------------------------------
template <typename LE, typename RE, typename HE>
class TrackingBase {
  private:
  //-1 for invalid, 0 for tank/encoders, 1 for X
  int baseDesign = -1;
  bool hasHor = false;
  
  void* left = NULL;
  void* right = NULL;
  void* hor = NULL;
  vex::motor* mFR = NULL;
  vex::motor* mFL = NULL;
  vex::motor* mBR = NULL;
  vex::motor* mBL = NULL;

  public:
  vex::inertial* inertialSensor = NULL;
  double width = 1;

  double globalAdj = 1;
  double leftK = 1;
  double rightK = 1;
  double horK = 1;

  Vector2d realitiveVector = Vector2d(0, 0);
  double head = 0;

  double horEncPrev = 0;
  double leftEncPrev = 0;
  double rightEncPrev = 0;

  double mFRPrev = 0;
  double mFLPrev = 0;
  double mBRPrev = 0;
  double mBLPrev = 0;

  //Encoder get functions
  void resetRightEnc() {
      if (right == NULL) {
          return;
      }
      (*(RE*)right).setPosition(0, vex::rotationUnits::rev);
      rightEncPrev = 0;
  }
  double getRightEnc() {
      if (right == NULL) {
          return 0;
      }
      return globalAdj * rightK * (*(RE*)right).position(vex::rotationUnits::rev);
  }

  void resetLeftEnc() {
      if (left == NULL) {
          return;
      }
      (*(RE*)left).setPosition(0, vex::rotationUnits::rev);
      leftEncPrev = 0;
  }
  double getLeftEnc() {
      if (left == NULL) {
          return 0;
      }
      return globalAdj * leftK * (*(LE*)left).position(vex::rotationUnits::rev);
  }

  void resetHorEnc() {
      if (hor == NULL) {
          return;
      }
      (*(RE*)hor).setPosition(0, vex::rotationUnits::rev);
      horEncPrev = 0;
  }
  double getHorEnc() {
      if (hor == NULL) {
          return 0;
      }
      return globalAdj * horK * (*(HE*)hor).position(vex::rotationUnits::rev);
  }
  
  //X Drive get functions
  void resetFREnc() {
      if (mFR == NULL) {
          return;
      }
      (*mFR).setPosition(0, vex::rotationUnits::rev);
      mFRPrev = 0;
  }
  double getFREnc() {
      if (mFR == NULL) {
          return 0;
      }
      return globalAdj * (*mFR).position(vex::rotationUnits::rev);
  }

  void resetFLEnc() {
      if (mFL == NULL) {
          return;
      }
      (*mFL).setPosition(0, vex::rotationUnits::rev);
      mFLPrev = 0;
  }
  double getFLEnc() {
      if (mFL == NULL) {
          return 0;
      }
      return globalAdj * (*mFL).position(vex::rotationUnits::rev);
  }

  void resetBLEnc() {
      if (mBL == NULL) {
          return;
      }
      (*mBL).setPosition(0, vex::rotationUnits::rev);
      mBLPrev = 0;
  }
  double getBLEnc() {
      if (mBL == NULL) {
          return 0;
      }
      return globalAdj * (*mBL).position(vex::rotationUnits::rev);
  }

  void resetBREnc() {
      if (mBR == NULL) {
          return;
      }
      (*mBR).setPosition(0, vex::rotationUnits::rev);
      mBRPrev = 0;
  }
  double getBREnc() {
      if (mBR == NULL) {
          return 0;
      }
      return globalAdj * (*mBR).position(vex::rotationUnits::rev);
  }

public:
    //Encoder/Tank Constructor w/ Inertial
    TrackingBase(LE* leftEncIn, bool isLeftReversed, RE* rightEncIn, bool isRightReversed,
        double encoderWidth, vex::inertial* inertialSensorin = NULL, HE* horEncIn = NULL, bool isHorReversed = false) {
        left = leftEncIn;
        right = rightEncIn;
        hor = horEncIn;
        inertialSensor = inertialSensorin;
        width = encoderWidth;

        if (isLeftReversed) {
            leftK = -1;
        }
        if (isRightReversed) {
            rightK = -1;
        }
        if (isHorReversed) {
            horK = -1;
        }

        if (right != NULL && left != NULL) {
            baseDesign = 0;
        }

        if (horEncIn != NULL) {
            hasHor = true;
        }
    }

  //X Drive Constructor
  TrackingBase(vex::motor* frontRight, vex::motor* frontLeft,
      vex::motor* backRight, vex::motor* backLeft, double encoderWidth, vex::inertial* inertialSensorin = NULL) {
      mFR = frontRight;
      mFL = frontLeft;
      mBR = backRight;
      mBL = backLeft;
      inertialSensor = inertialSensorin;
      width = encoderWidth;

      if (mFR != NULL && mFL != NULL && mBR != NULL && mBL != NULL) {
          baseDesign = 1;
      }
  }

  //Set Coefficents
  void setGlobalCoefficent(double k) { globalAdj = fabs(k); }
  void setLeftCoefficent(double k) { leftK = sign(leftK) * k; }
  void setRightCoefficent(double k) { rightK = sign(rightK) * k; }
  void setHorCoefficent(double k) { horK = sign(horK) * k; }

  //Sets heading
  void setHeading(double newHead, bool inDeg = true) {
      newHead = normalizeAngle(newHead, !inDeg);

      if (inDeg) {
          if(inertialSensor != NULL) {
              inertialSensor->setHeading(360 - newHead, vex::rotationUnits::deg);
          }
          
          newHead = degToRad(newHead);
      }
      else {
          if (inertialSensor != NULL) {
              inertialSensor->setHeading(360 - radToDeg(newHead), vex::rotationUnits::deg);
          }
      }

      head = newHead;
  }

  //Resets all encoders - Recommended only use during tracking setup
  void resetAll() {
      //Resets encoders and prevEncoders
      resetRightEnc();
      resetLeftEnc();
      resetHorEnc();

      resetBLEnc();
      resetBREnc();
      resetFLEnc();
      resetFREnc();
  }

  //Updates heading and positionDeltas
  void update() {
      static bool notLoggedFailure = true;
      if (inertialSensor != NULL) {
          if (inertialSensor->installed()) {
              //Inertial OK
              double headTemp = inertialSensor->angle(); //Gets unbounded angle; Positive in CW direction
              headTemp = 360 - normalizeAngle(headTemp, false); //Heading in CCW + direction
              notLoggedFailure = true;
              head = degToRad(headTemp);
          }
      }
      else {
          //No Inertial
          if (notLoggedFailure) {
              coreLogger.error("INERTIAL FAILURE", "TRACKINGBASE");
              notLoggedFailure = false;
          }

          if (baseDesign == 0) {
              head += ((getRightEnc() - rightEncPrev) - (getLeftEnc() - leftEncPrev)) / width;
          }
          else if (baseDesign == 1) {
              double a = (getFREnc() - mFRPrev) - (getBLEnc() - mBLPrev);
              a += (getBREnc() - mBRPrev) - (getFLEnc() - mFLPrev);
              head += a / (2 * width); //Average and divide by 2r (2r=width) to get angular turn
          }

      }

      if (baseDesign == 0) {
          double deltaForward = 0;
          double deltaHorizontal = 0;

          //Tank
          double left = getLeftEnc();
          double right = getRightEnc();
          deltaForward = 0.5 * (left - leftEncPrev + right - rightEncPrev);

          leftEncPrev = left;
          rightEncPrev = right;

          left = getHorEnc(); //Reuse left to save memory
          deltaHorizontal = left - horEncPrev;
          horEncPrev = left;

          realitiveVector = Vector2d(deltaHorizontal, deltaForward);
      }
      else if (baseDesign == 1) {
          //X Drive

          double A = 0.5 * (getFREnc() - mFRPrev + getBLEnc() - mBLPrev);
          double B = 0.5 * (getBREnc() - mBRPrev + getFLEnc() - mFLPrev);

          mFRPrev = getFREnc();
          mFLPrev = getFLEnc();
          mBRPrev = getBREnc();
          mBLPrev = getBLEnc();

          realitiveVector = Vector2d(A, 135, true) + Vector2d(B, 45, true);
      }
  }

  //Returns heading from inertial sensor or wheels if inertial failed or non present
  double getHeading() {
      return head;
  }
  
  //Returns a vector from the robot's frame of reference
  Vector2d getRelVector() {
      return realitiveVector;
  }

  //Returns a vector from the field's frame of reference
  Vector2d getAbsVector() {      
      return realitiveVector.getRotatedVector(getHeading() - M_PI_2);
  }
};

//Events
//--------------------------------------------------------------------------------------------------
class NavigationEvent {
    //Holds details about
};

class EventManager {
    //Handles events, sorts into list based on positon, binary search/metadata memory search to find closest events to call, removes events when they should no longer be called, remembers order to call events,
    //thread safe
};

//Navigator
//--------------------------------------------------------------------------------------------------
typedef struct{
  Point2d currentPosition;
  double currentHeading;
  bool currentHeadingInDeg;
}startingPosition;

class Navigator{
private:
    const double STOP_TIME = 0.2;
    double linearStopRadius = 1;
    double angularStopRadius = degToRad(1);
    
    double linearStopTimer = STOP_TIME;
    double rotationalStopTimer = STOP_TIME;

    bool linearStopped = true;
    bool rotationalStopped = true;

    positionSet currentPos = { Point2d(0, 0), 0 };
    positionSet previousPos = { Point2d(0, 0), 0 }; //Used only for velocity calculations in update()
    positionSet lastStoppedPos = { Point2d(0, 0), 0 };
    positionSet lastTarget = { Point2d(0, 0), 0 };

    Vector2d velocity = Vector2d(0, 0);
    Vector2d acceleration = Vector2d(0, 0);
    double angularVelocity = 0;
    double angularAcceleration = 0;

    int lastPointCap = 3;
    Path previousPath; //TODO - Add timing component incase loops become inconsistant?
    Path targetPath; //TODO

    void updateStopTime(double deltaTime) {
        //Linear
        if (getVelocity().getMagnitude() * STOP_TIME < linearStopRadius) {
            if (linearStopTimer < STOP_TIME) {
                linearStopTimer += deltaTime;
            }
            else {
                lastStoppedPos.p = currentPos.p;
                linearStopped = true;
            }
        }
        else {
            linearStopped = false;
            linearStopTimer = 0;
        }

        //Rotation
        if (abs(getAngularVelocity()) * STOP_TIME < angularStopRadius) {
            if (rotationalStopTimer < STOP_TIME) {
                rotationalStopTimer += deltaTime;
            }
            else {
                lastStoppedPos.head = currentPos.head;
                rotationalStopped = true;
            }
        }
        else {
            rotationalStopped = false;
            rotationalStopTimer = 0;
        }
    }

public:
    Navigator() {
        //stopRadius needs to be init
        //Starting pos needs to be init
    }

    //Set starting Position
    void setStartingPos(Point2d currentPosition, double currentHeading, bool currentHeadingInDeg) {
        setCurrentPosition(currentPosition);
        setHead(currentHeading, currentHeadingInDeg);
        previousPos = currentPos;
        lastStoppedPos = currentPos;
        lastTarget = currentPos;
    }

    void setStartingPos(startingPosition pos) {
        setStartingPos(pos.currentPosition, pos.currentHeading, pos.currentHeadingInDeg);
    }

    //Current Position Setters
    void setCurrentPosition(Point2d p) {
        currentPos.p = p;
    }

    void shiftCurrentPosition(Vector2d shift) {
        currentPos.p = currentPos.p + shift;
    }

    void setHead(double newHead, bool headingInDegrees) {
        if (headingInDegrees) {
            newHead = degToRad(newHead);
        }
        currentPos.head = newHead;
    }

    void shiftCurrentHead(double shift, bool headingInDegrees) {
        if (headingInDegrees) { shift = degToRad(shift); }
        currentPos.head = currentPos.head + shift;
    }

    void setLastPointCap(int num) {
        lastPointCap = max(3, num);
    }

    //Update based on currentPosition and time
    void updateNavigation(double deltaTime) {
        double inverseDeltaTime = 1 / deltaTime;

        Vector2d newVel = (currentPos.p - previousPos.p).scale(inverseDeltaTime);
        acceleration = (newVel - velocity).scale(inverseDeltaTime);
        velocity = newVel;

        double newAngularVel = (currentPos.head - previousPos.head) * inverseDeltaTime; //ASSUMES NON-NORMLIZED HEADING
        angularAcceleration = (newAngularVel - angularVelocity) * inverseDeltaTime;
        angularVelocity = newAngularVel;

        updateStopTime(deltaTime);

        //TODO Target management, events, etc
        //Hanndle Tagrte management, calculating arclength, curvature, nextTagrte, target vector, etc; Motion contoller decides when next tagret is, navigation just answers question about the path

        if (previousPath.size() >= lastPointCap) {
            previousPath.drop(1);
        }
        previousPath.addPointset(currentPos);
        previousPos = currentPos;
    }

    void clearTargets() {
        targetPath.clear();
    }

    void shiftTarget() {
        targetPath.next();
    }

    void addTarget(double x, double y) {
        Point2d p = Point2d(x, y);
        lastTarget = { p, normalizeAngle(Vector2d(1, 0).getAngle(Vector2d(lastTarget.p, p))) };
        targetPath.addPointset(lastTarget);
    }

    void addTarget(double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); }
        heading = normalizeAngle(heading);
        lastTarget = { lastTarget.p, heading };
        targetPath.addPointset(lastTarget);
    }

    void addTarget(double x, double y, double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); }
        lastTarget = { Point2d(x, y), heading };
        targetPath.addPointset(lastTarget);
    }

    void addTarget(positionSet in, bool inDeg = true) {
        if (inDeg) { in.head = degToRad(in.head); }
        in.head = normalizeAngle(in.head);
        lastTarget = in;
        targetPath.addPointset(in);
    }

    void addTarget(Vector2d in) {
        addTarget(in.getX(), in.getY());
    }

    void addRelTarget(Vector2d in) {
        in = translateLocalToGlobal(in);
        addTarget(in.getX(), in.getY());
    }

    void addRelTarget(double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); }
        lastTarget = { lastTarget.p, normalizeAngle(currentPos.head + heading) };
        targetPath.addPointset(lastTarget);
    }

    //Getters
    positionSet getPosition() {
        return currentPos;
    }

    positionSet getLastStoppedPos() {
        return lastStoppedPos;
    }

    int getTargetIndex() {
        return targetPath.index();
    }

    positionSet getTarget() {
        return targetPath[getTargetIndex()];
    }

    positionSet getNextTarget() {
        return targetPath[getTargetIndex() + 1];
    }

    Path& getPreviousPath() {
        return previousPath;
    }

    Path& getTargetPath() {
        return targetPath;
    }

    Vector2d getVelocity() {
        return velocity;
    }

    Vector2d getAcceleration() {
        return acceleration;
    }

    double getAngularVelocity() {
        return angularVelocity;
    }

    double getAngularAcceleration() {
        return angularAcceleration;
    }

    Vector2d getRobotNormalVector() {
        return Vector2d(1, normalizeAngle(currentPos.head), false);
    }

    Vector2d translateGlobalToLocal(Vector2d v) {
        return v.getRotatedVector(M_PI_2 - normalizeAngle(currentPos.head));
    }
    Vector2d translateLocalToGlobal(Vector2d v) {
        return v.getRotatedVector(normalizeAngle(currentPos.head) - M_PI_2);
    }

    bool isLinearStopped() {
        return linearStopped;
    }

    bool isRotationalStopped() {
        return rotationalStopped;
    }

    double currentRadiusOfCurvature(){
        double v = getVelocity().getMagnitude();
        double a = getVelocity().getUnitVector().cross(getAcceleration());
        if (abs(a) < 0.0001) { return 0; }
        return (v * v / a);
    }
};

extern Navigator navigation;

#endif