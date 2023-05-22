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
TrackingBase<LE, RE, HE>::TrackingBase(LE *leftEncIn, bool leftReversed, RE *rightEncIn, bool rightReversed, 
                            double encoderWidth, vex::inertial *inertialSensorin, HE *horEncIn, bool horReversed) {
  left = leftEncIn;
  right = rightEncIn;
  hor = horEncIn;
  inertialSensor = inertialSensorin;
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

  if(horEncIn != NULL){
    hasHor = true;
  }
}

template <typename LE, typename RE, typename HE>
TrackingBase<LE, RE, HE>::TrackingBase(vex::motor *frontRight, vex::motor *frontLeft,
                            vex::motor *backRight, vex::motor *backLeft, double encoderWidth, vex::inertial *inertialSensorin) {
  mFR = frontRight;
  mFL = frontLeft;
  mBR = backRight;
  mBL = backLeft;
  inertialSensor = inertialSensorin;
  width = encoderWidth;
  
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
void TrackingBase<LE, RE, HE>::setHeading(double newHead, bool inDeg) { 
  newHead = normalizeAngle(newHead, !inDeg);

  if(inDeg){
    if(inertialSensor != NULL){
      inertialSensor->setRotation(360 - newHead, vex::rotationUnits::deg);
    }

    newHead = degToRad(newHead);
  } else {
    if(inertialSensor != NULL){
      inertialSensor->setRotation(360 - radToDeg(newHead), vex::rotationUnits::deg);
    }
  }

  head = newHead;
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::resetAll() {
  //Resets encoders and prevEncoders
  resetRightEnc();
  resetLeftEnc();
  resetHorEnc();

  resetBLEnc();
  resetBREnc();
  resetFLEnc();
  resetFREnc();
}

template <typename LE, typename RE, typename HE>
void TrackingBase<LE, RE, HE>::update() {
  static bool notLoggedFailure = true;
  if(inertialSensor != NULL && inertialSensor->installed()){
      //Inertial OK
      double headTemp = inertialSensor->angle(); //Gets unbounded angle; Positive in CW direction
      headTemp = 360 - normalizeAngle(headTemp, false); //Heading in CCW + direction
      notLoggedFailure = true;
      head = degToRad(headTemp);
  } else {
    //No Inertial
    if(notLoggedFailure){
      coreLogger.error("INERTIAL FAILURE", "TRACKINGBASE");
      notLoggedFailure = false;
    }

    if(baseDesign == 0){
      head += ((getRightEnc() - rightEncPrev) - (getLeftEnc() - leftEncPrev)) / width;
    }else if(baseDesign == 1){
      double a = (getFREnc() - mFRPrev) - (getBLEnc() - mBLPrev);
      a += (getBREnc() - mBRPrev) - (getFLEnc() - mFLPrev);
      head += a / (2 * width); //Average and divide by 2r (2r=width) to get angular turn
    }

  }

  if(baseDesign == 0){
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

    realitiveVector = Vector2d(deltaForward, deltaHorizontal);
  }else if(baseDesign == 1){
    //X Drive

    double A =  0.5*(getFREnc() - mFRPrev + getBLEnc() - mBLPrev);
    double B =  0.5*(getBREnc() - mBRPrev + getFLEnc() - mFLPrev);

    mFRPrev = getFREnc();
    mFLPrev = getFLEnc();
    mBRPrev = getBREnc();
    mBLPrev = getBLEnc();

    realitiveVector = Vector2d(A, 135, true) + Vector2d(B, 45, true);
  }
}


// Returns heading from inertial sensor or wheels if inertial failed or non present
template <typename LE, typename RE, typename HE>
double TrackingBase<LE, RE, HE>::getHeading() {
  return head;
}

// Returns a vector from the robot's frame of reference
template <typename LE, typename RE, typename HE>
Vector2d TrackingBase<LE, RE, HE>::getRelVector() {
  return realitiveVector;
}

// Returns a vector from the field's frame of reference
template <typename LE, typename RE, typename HE>
Vector2d TrackingBase<LE, RE, HE>::getAbsVector() {
  return Vector2d(realitiveVector.getMagnitude(), getHeading(), false);
}
