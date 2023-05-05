#include "v5.h"
#include "v5_vcs.h"
#include "tracking/logger.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

using namespace vex;

bool RemoteControlCodeEnabled = true;

//Basic
brain Brain;
controller Controller1 = controller(primary);

//Motors
motor backFlywheel = motor(PORT18, ratio6_1, true);
motor frontFlywheel = motor(PORT15, ratio6_1, false);

motor frontIntake = motor(PORT1,ratio6_1,true);
motor Roller1 = motor(PORT16,ratio6_1,true);

motor EndgameML = motor(PORT11,ratio36_1,false);
motor EndgameMR = motor(PORT19,ratio36_1,true);
motor trigger = motor(PORT17, true);

motor leftMotorFrontTop = motor(PORT4, false);
motor leftMotorFrontBottom = motor(PORT5, true);
motor leftMotorBackTop = motor(PORT7, false);
motor leftMotorBackBottom = motor(PORT8, true);

motor rightMotorFrontTop = motor(PORT6, true);
motor rightMotorFrontBottom = motor(PORT13, false);
motor rightMotorBackTop = motor(PORT9, true);
motor rightMotorBackBottom = motor(PORT10, false);


//Motor Groups
motor_group leftDriveMotorGroup = motor_group(leftMotorFrontTop, leftMotorFrontBottom, leftMotorBackTop, leftMotorBackBottom);
motor_group rightDriveMotorGroup = motor_group(rightMotorFrontTop, rightMotorFrontBottom, rightMotorBackTop, rightMotorBackBottom);
motor_group FlywheelGroup = motor_group(frontFlywheel, backFlywheel);
motor_group Endgame = motor_group(EndgameML, EndgameMR);

//Sensors
inertial Inertial = inertial(PORT14);
encoder leftEnc = encoder(Brain.ThreeWirePort.G);
encoder rightEnc = encoder(Brain.ThreeWirePort.E);
encoder horEnc = encoder(Brain.ThreeWirePort.A);

digital_out endGamePne = digital_out(Brain.ThreeWirePort.C);

void setupMotors(){
  FlywheelGroup.setStopping(brakeType::coast);

  frontIntake.setStopping(brakeType::brake);
  Roller1.setStopping(brakeType::hold);
  
  rightDriveMotorGroup.setStopping(brakeType::brake);
  leftDriveMotorGroup.setStopping(brakeType::brake);
}

double getHeadingUnbounded(){
  double head = Inertial.angle();
  return head;
}

double getHeading(){
  double toss;
  double r = (360)*modf(getHeadingUnbounded()/(360), &toss);
  if(r<0){r+=(360);}
  return r;
}

double getHeadingCCW(){
  return (360 - getHeading());
}

void resetEncoders(){
  leftEnc.setPosition(0, rotationUnits::rev);
  rightEnc.setPosition(0, rotationUnits::rev);
  horEnc.setPosition(0, rotationUnits::rev);
}

double getRightEnc(){
  return rightEnc.position(rotationUnits::rev);
}

double getLeftEnc(){
  return -1 * leftEnc.position(rotationUnits::rev);
}

double getHorEnc(){
  return horEnc.position(rotationUnits::rev);
}

void setGroupVoltage(motor_group &group, double volts){
  if(volts < 0){
    volts = 0;
  }
  if(volts > 12){
    volts = 12;
  }
  if(volts < 0.1){
    group.stop();
  }else{
    group.spin(directionType::fwd, volts, voltageUnits::volt);
  }
}

class Flywheel {
private:
  double lastTime = 0;
  double setValue = 0;
  double lastError = 0;
  double deltaError = 0;
  
  
  double output = 0;
  motor_group* flyGroup;

  double target = 0;
  bool enabled = false;
  bool activeAdjust = false;

  const double cycleTimeMsec = 12;

  const static int speedCycles = 3;
  double speeds[speedCycles];

  void addSpeed(double s){
    for(int i = 0; i < speedCycles - 1; i++){
      speeds[i] = speeds[i + 1];
    }
    speeds[speedCycles - 1] = s;
  }

  double calcVoltageForSpeed(double speed){
    return (speed + 17.75) / 9.345;
  }
/*
  double predictNextSpeed(){
    //https://mathworld.wolfram.com/LagrangeInterpolatingPolynomial.html
    double result = 0;
    for(int j = 0; j < speedCycles; j++){
      double coefficent = 1;
      double y = speeds[j];
      for(int i = 0; i < speedCycles; i++){
        if(i != j){
          coefficent *= (speedCycles - i) / (j - i);
        }
      }
      result += coefficent * y;
    }
    return result;
  }*/

public:
  double kP = 0;
  double kI = 0;
  double kD = 0;
  const double UPDATE_ERROR_BOUNDS = 10;

  Flywheel(motor_group* flyGroupObj, double Pk, double Ik, double Dk) {
    flyGroup = flyGroupObj;
    kP = Pk;
    kI = Ik;
    kD = Dk;
  }

  void enable(){
    setTarget(target);
    enabled = true;
  }

  void disable(){
    enabled = false;
  }

  void clearSpeeds(){
    double s = getSpeed();
    for(int i = 0; i < speedCycles; i++){
      speeds[i] = s;
    }
  }

  void clearSpeeds(double speedIn){
    for(int i = 0; i < speedCycles; i++){
      speeds[i] = speedIn;
    }
  }

  void setTarget(double targetSpeed){
    target = targetSpeed;
    setValue = calcVoltageForSpeed(targetSpeed);
    deltaError = 0;
    activeAdjust = false;
  }

  double getSpeed(){
    return (*flyGroup).velocity(percentUnits::pct);
  }

  double getAvgSpeed(){
    double sum = 0;
    for(int i=0; i<speedCycles; i++){
      sum += speeds[i];
    }
    return sum / speedCycles;
  }


  void update() { 
    addSpeed(getSpeed());
    double error = target - getAvgSpeed();
    double deltaTime = (Brain.timer(timeUnits::msec) - lastTime) / 1000.0;

    //std::cout << Brain.timer(msec) << ", " << FlywheelGroup.velocity(pct) << ", " << error << ", " << kP*error << ", " << setValue << std::endl;
    
    if(enabled){
      if(fabs(error) < UPDATE_ERROR_BOUNDS){
        if(!activeAdjust){
          activeAdjust = true;
        }
      }

      output = 0;

      //Control P
      if(activeAdjust){
        output += kP * error;
      }

      //Control I
      if(activeAdjust){
        setValue += kI * error * deltaTime;
        if(setValue < 0){setValue=0;}
        else if (setValue > 12) {setValue = 12;}
      }
      output += setValue;

      //Control D
      if(activeAdjust){
        deltaError = error - lastError;
        output += kD * deltaError / deltaTime;
      }

      //Set P+I+D
      if(output > 12){output = 12;}
      if(output < 0){output = 0;}
      setGroupVoltage(*flyGroup, output);
    }
    
    lastError = error;
    lastTime = Brain.timer(timeUnits::msec);
    wait(cycleTimeMsec, msec);
  }

  bool fireReady(){
    if (fabs(target - getSpeed()) < 4){
      return true;
    }
    return false;
  }
};
