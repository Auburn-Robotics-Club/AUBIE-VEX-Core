#ifndef ROBOT_NAV_H
#define ROBOT_NAV_H

#include "robotmath.h"
#include "logger.h"
#include "../v5.h"
#include "../v5_vcs.h"

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
              inertialSensor->setRotation(360 - newHead, vex::rotationUnits::deg);
          }

          newHead = degToRad(newHead);
      }
      else {
          if (inertialSensor != NULL) {
              inertialSensor->setRotation(360 - radToDeg(newHead), vex::rotationUnits::deg);
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
      return normalizeAngle(head);
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

typedef struct{
  Point2d currentPosition;
  double currentHeading;
  bool currentHeadingInDeg;
}startingPosition;

class Navigator{
  //
};

extern Navigator navigation;

#endif