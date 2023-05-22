#include "AUBIE-VEX-CORE/navigation.h"

// TrackingBase
//--------------------------------------------------------------------------------------------------
template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetRightEnc() {
  if (right == NULL) {
    return;
  }
  (*(RE *)right).setPosition(0, vex::rotationUnits::rev);
  rightEncPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getRightEnc() {
  if (right == NULL) {
    return 0;
  }
  return globalAdj * rightK * (*(RE *)right).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetLeftEnc() {
  if (left == NULL) {
    return;
  }
  (*(RE *)left).setPosition(0, vex::rotationUnits::rev);
  leftEncPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getLeftEnc() {
  if (left == NULL) {
    return 0;
  }
  return globalAdj * leftK * (*(LE *)left).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetHorEnc() {
  if (hor == NULL) {
    return;
  }
  (*(RE *)hor).setPosition(0, vex::rotationUnits::rev);
  horEncPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getHorEnc() {
  if (hor == NULL) {
    return 0;
  }
  return globalAdj * horK * (*(HE *)hor).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetFREnc() {
  if (mFR == NULL) {
    return;
  }
  (*mFR).setPosition(0, vex::rotationUnits::rev);
  mFRPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getFREnc() {
  if (mFR == NULL) {
    return 0;
  }
  return globalAdj * (*mFR).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetFLEnc() {
  if (mFL == NULL) {
    return;
  }
  (*mFL).setPosition(0, vex::rotationUnits::rev);
  mFLPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getFLEnc() {
  if (mFL == NULL) {
    return 0;
  }
  return globalAdj * (*mFL).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetBLEnc() {
  if (mBL == NULL) {
    return;
  }
  (*mBL).setPosition(0, vex::rotationUnits::rev);
  mBLPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getBLEnc() {
  if (mBL == NULL) {
    return 0;
  }
  return globalAdj * (*mBL).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetBREnc() {
  if (mBR == NULL) {
    return;
  }
  (*mBR).setPosition(0, vex::rotationUnits::rev);
  mBRPrev = 0;
}

template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getBREnc() {
  if (mBR == NULL) {
    return 0;
  }
  return globalAdj * (*mBR).position(vex::rotationUnits::rev);
}

template <typename LE, typename RE, typename HE>
TrackingBase<LE, RE, HE>::TrackingBase(LE *leftEncIn, bool leftReversed, RE *rightEncIn,
                            bool rightReversed, vex::inertial *inertialSensorin,
                            HE *horEncIn,bool horReversed) {
  left = leftEncIn;
  right = rightEncIn;
  hor = horEncIn;
  inertialSensor = inertialSensorin;
  if (leftReversed) {
    leftK = -1;
  }
  if (rightReversed) {
    rightReversed = -1;
  }
  if (horReversed) {
    horReversed = -1;
  }
  
  if(right != NULL && left != NULL){
    baseDesign = 0;
  }
}

template <typename LE, typename RE, typename HE>
TrackingBase<LE, RE, HE>::TrackingBase(LE *leftEncIn, bool leftReversed, RE *rightEncIn,
                            bool rightReversed, double encoderWidth, HE *horEncIn, bool horReversed) {
  left = leftEncIn;
  right = rightEncIn;
  hor = horEncIn;
  width = encoderWidth;
  if (leftReversed) {
    leftK = -1;
  }
  if (rightReversed) {
    rightReversed = -1;
  }
  if (horReversed) {
    horReversed = -1;
  }
  
  if(right != NULL && left != NULL){
    baseDesign = 0;
  }
}

template <typename LE, typename RE, typename HE>
TrackingBase<LE, RE, HE>::TrackingBase(vex::motor *frontRight, vex::motor *frontLeft,
                            vex::motor *backRight, vex::motor *backLeft, vex::inertial *inertialSensorin) {
  vex::motor *mFR = frontRight;
  vex::motor *mFL = frontLeft;
  vex::motor *mBR = backRight;
  vex::motor *mBL = backLeft;
  inertialSensor = inertialSensorin;
  
  if(mFR != NULL && mFL != NULL && mBR != NULL && mBL != NULL){
    baseDesign = 1;
  }
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::setGlobalCoefficent(double k) { globalAdj = fabs(k); }

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::setLeftCoefficent(double k) { leftK = sign(leftK) * k; }

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::setRightCoefficent(double k) { rightK = sign(rightK) * k; }

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::setHorCoefficent(double k) { horK = sign(horK) * k; }

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetAll() {
  resetRightEnc();
  resetLeftEnc();
  resetHorEnc();

  resetBLEnc();
  resetBREnc();
  resetFLEnc();
  resetFREnc();
}

// Returns heading from inertial sensor or wheels if inertial failed or non present
template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getHeading() {
  if(inertialSensor != NULL){
    if(inertialSensor->installed()){
      if(!inertialSensor->isCalibrating()){
        //Inertial OK
        double head = inertialSensor->angle(); //Gets unbounded angle; Positive in CW direction
        head = 360 - normalizeAngle(head, false); //Heading in CCW + direction
        return head;
      }
    }
  }
  //No Inertial
  if(baseDesign == 0){
    //Tank

  }else if(baseDesign == 1){
    //X Drive

  }
}

// Returns a vector from the robot's frame of reference
template <typename LE, typename RE, typename HE>
Vector2d TrackingBase<LE, RE, HE>::getRelVector() {
  double deltaForward = 0;
  if(baseDesign == 0){
    //Tank

  }else if(baseDesign == 1){
    //X Drive
    
  }

  double deltaHorizontal = 0;
  if(hor != NULL){
    deltaHorizontal = getHorEnc();
  }
  return Vector2d(deltaForward, deltaHorizontal);
}

// Returns a vector from the field's frame of reference
template <typename LE, typename RE, typename HE>
Vector2d TrackingBase<LE, RE, HE>::getAbsVector() {
  // TODO getRelVector * heading;
  //TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}
