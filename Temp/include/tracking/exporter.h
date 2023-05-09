// Look into these functions found in v5_api.h
  //vexDeviceGetByIndex(0);
  //vexSerialReadChar(9600);
#pragma once
#include "robotmath.h"

template <typename LE, typename RE, typename HE>
class TrackingBase {
private:
  void* left = NULL;
  void* right = NULL;
  void* hor = NULL;
  vex::motor* mFR = NULL;
  vex::motor* mFL = NULL;
  vex::motor* mBR = NULL;
  vex::motor* mBL = NULL;
  vex::inertial* inertialSensor = NULL;
  double width = 1;

  double leftK = 1;
  double rightK = 1;
  double horK = 1;

  double horEncPrev = 0;
  double leftEncPrev = 0;
  double rightEncPrev = 0;

  double mFRPrev = 0;
  double mFLPrev = 0;
  double mBRPrev = 0;
  double mBLPrev = 0;

public:
  double globalAdj = 1;

  TrackingBase(LE* leftEncIn, bool leftReversed, RE* rightEncIn, bool rightReversed, vex::inertial* inertialSensorin, HE* horEncIn=NULL, bool horReversed = false){
    left = leftEncIn;
    right = rightEncIn;
    hor = horEncIn;
    inertialSensor = inertialSensorin;
    if(leftReversed){leftK=-1;}
    if(rightReversed){rightReversed=-1;}
    if(horReversed){horReversed=-1;}
  }

  TrackingBase(LE* leftEncIn, bool leftReversed, RE* rightEncIn, bool rightReversed, double encoderWidth, HE* horEncIn=NULL, bool horReversed = false){
    left = leftEncIn;
    right = rightEncIn;
    hor = horEncIn;
    width = encoderWidth;
    if(leftReversed){leftK=-1;}
    if(rightReversed){rightReversed=-1;}
    if(horReversed){horReversed=-1;}
  }

  TrackingBase(vex::motor* frontRight, vex::motor* frontLeft, vex::motor* backRight, vex::motor* backLeft, vex::inertial* inertialSensorin){
    vex::motor* mFR = NULL;
    vex::motor* mFL = NULL;
    vex::motor* mBR = NULL;
    vex::motor* mBL = NULL;
    inertialSensor = inertialSensorin;
  }

  void setLeftCoefficent(double k){
    leftK = sign(leftK) * k;
  }

  void setRightCoefficent(double k){
    rightK = sign(rightK) * k;
  }

  void setHorCoefficent(double k){
    horK = sign(horK) * k;
  }

  double getHeadingSensorUnbounded(){
    if(inertialSensor != NULL) {
      double head = inertialSensor->angle();
      return head;
    }
    return 0.0;
  }

  void resetRightEnc(){
    if(right == NULL){
      return;
    }
    (*(RE*)right).setPosition(0, vex::rotationUnits::rev);
    rightEncPrev = 0;
  }

  double getRightEnc() {
    if(right == NULL){
      return 0;
    }
    return globalAdj * rightK * (*(RE*)right).position(vex::rotationUnits::rev);
  }

  void resetLeftEnc(){
    if(left == NULL){
      return;
    }
    (*(RE*)left).setPosition(0, vex::rotationUnits::rev);
    leftEncPrev = 0;
  }

  double getLeftEnc() {
    if(left == NULL){
      return 0;
    }
    return globalAdj * leftK * (*(LE*)left).position(vex::rotationUnits::rev);
  }

  void resetHorEnc(){
    if(hor == NULL){
      return;
    }
    (*(RE*)hor).setPosition(0, vex::rotationUnits::rev);
    horEncPrev = 0;
  }

  double getHorEnc() {
    if(hor == NULL){
      return 0;
    }
    return globalAdj * horK * (*(HE*)hor).position(vex::rotationUnits::rev);
  }

  void resetFREnc(){
    if(mFR == NULL){
      return;
    }
    (*mFR).setPosition(0, vex::rotationUnits::rev);
    mFRPrev = 0;
  }

  double getFREnc() {
    if(mFR == NULL){
      return 0;
    }
    return globalAdj * (*mFR).position(vex::rotationUnits::rev);
  }

  void resetFLEnc(){
    if(mFL == NULL){
      return;
    }
    (*mFL).setPosition(0, vex::rotationUnits::rev);
    mFLPrev = 0;
  }

  double getFLEnc() {
    if(mFL == NULL){
      return 0;
    }
    return globalAdj * (*mFL).position(vex::rotationUnits::rev);
  }

  void resetBLEnc(){
    if(mBL == NULL){
      return;
    }
    (*mBL).setPosition(0, vex::rotationUnits::rev);
    mBLPrev = 0;
  }

  double getBLEnc() {
    if(mBL == NULL){
      return 0;
    }
    return globalAdj * (*mBL).position(vex::rotationUnits::rev);
  }

  void resetBREnc(){
    if(mBR == NULL){
      return;
    }
    (*mBR).setPosition(0, vex::rotationUnits::rev);
    mBRPrev = 0;
  }

  double getBREnc() {
    if(mBR == NULL){
      return 0;
    }
    return globalAdj * (*mBR).position(vex::rotationUnits::rev);
  }

  void resetAll(){
    resetRightEnc();
    resetLeftEnc();
    resetHorEnc();

    resetBLEnc();
    resetBREnc();
    resetFLEnc();
    resetFREnc();
  }

  void updateLocation(){
    //Update stuff and the get will jsut return the updates
  }

  double getDeltaHeading(){

  }
  
  Vector2d getDeltaVector(){
    
  }
/*
  //Gets the current heading either by sensor or by updating heading when senspor is null or broken
  double getHeading(){
    double toss;
    double r = (360)*modf(getHeadingUnbounded()/(360), &toss);
    if(r<0){r+=(360);}
    return r;
  }

  double getHeadingCCW(){
    return (360 - getHeading());
  }*/
};
