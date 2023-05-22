#pragma once

#include "robotmath.h"
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

  void* left = NULL;
  void* right = NULL;
  void* hor = NULL;
  vex::motor* mFR = NULL;
  vex::motor* mFL = NULL;
  vex::motor* mBR = NULL;
  vex::motor* mBL = NULL;

  vex::inertial* inertialSensor = NULL;
  double width = 1;

  double globalAdj = 1;
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

  //Encoder get functions
  void resetRightEnc();
  double getRightEnc();

  void resetLeftEnc();
  double getLeftEnc();

  void resetHorEnc();
  double getHorEnc();
  
  //X Drive get functions
  void resetFREnc();
  double getFREnc();

  void resetFLEnc();
  double getFLEnc();

  void resetBLEnc();
  double getBLEnc();

  void resetBREnc();
  double getBREnc();

public:
  //Encoder/Tank Constructor w/ Inertial
  TrackingBase(LE* leftEncIn, bool leftReversed, RE* rightEncIn, bool rightReversed, vex::inertial* inertialSensorin, HE* horEncIn=NULL, bool horReversed = false);

  //Encoder/Tank Constructor wo/ Inertial
  TrackingBase(LE* leftEncIn, bool leftReversed, RE* rightEncIn, bool rightReversed, double encoderWidth, HE* horEncIn=NULL, bool horReversed = false);

  //X Drive Constructor
  TrackingBase(vex::motor* frontRight, vex::motor* frontLeft, vex::motor* backRight, vex::motor* backLeft, vex::inertial* inertialSensorin = NULL);

  //Set Coefficents
  void setGlobalCoefficent(double k);
  void setLeftCoefficent(double k);
  void setRightCoefficent(double k);

  //Sets horizontal tracking wheel coefficent
  void setHorCoefficent(double k);

  //Resets all encoders
  void resetAll();

  //Returns heading from inertial sensor or wheels if inertial failed or non present
  double getHeading();
  
  //Returns a vector from the robot's frame of reference
  Vector2d getRelVector();

  //Returns a vector from the field's frame of reference
  Vector2d getAbsVector();
};

class Path{
protected:
  int index = 0;
  std::vector<positionSet> points;

public:
  const std::vector<positionSet> * const getList();
  void setIndex(size_t newIndex=0); //If index > list.size set to list.size
  int size();
  bool hasNext();
  positionSet get();
  void drop(); //Delete upto and including current index then set index to 0

  void addPointset(positionSet Point);
  
  //For target:
  //Arclength for total and remaining path, generate points, read from file, event points, nextPoint
  //Given set of points interpolate
  
  //Smarter navigation
  //Store last couple points to measure derivitives
  //Predicitive motion
  //Extrapolate from points
};

class Navigator{
  //
};

Navigator navigation;
