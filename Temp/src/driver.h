#ifndef __DRIVER_H_
#define __DRIVER_H_
#include "implementation.h"


// TODO Interface driver setup with GUI programming

/*
Toggle example
void toggle(bool currState){
  static bool active = false;
  static bool press = false;
  if(currState != press){
    press = currState;
    if(currState){
      active = !active;

      if(active){//DO thing}else{//Do another thing}

    }
  }
}*/

const int ControllerDisplayUpdateTime = 2000;
void updateControllerDisplay(){
  static int lastT = Brain.Timer.time(timeUnits::msec);
  if(Brain.Timer.time(timeUnits::msec)-lastT > ControllerDisplayUpdateTime){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Battery: %d%", Brain.Battery.capacity());
      Controller1.Screen.newLine();

      double a = 0;
      a = fmax(a, leftDriveMotorGroup.temperature(temperatureUnits::celsius));
      a = fmax(a, rightDriveMotorGroup.temperature(temperatureUnits::celsius));

      Controller1.Screen.print("HighTemp: %.2f C", a);
      Controller1.Screen.newLine();
      Controller1.Screen.print("Flywheel: %.2f%", FlywheelGroup.velocity(percentUnits::pct));

      lastT = Brain.Timer.time(timeUnits::msec);
  }
}

int flyActive = 0;
int flySpeed = 0;
const int SPEEDHIGH = 85;
const int SPEEDLOW = 60;
void flywheelControl(){
  static bool xPressed = false;
  static bool yPressed = false;
  
  if(!xPressed && Controller1.ButtonX.pressing()){
    if(flySpeed == SPEEDHIGH){
      flyActive = 0;
      flySpeed = 0;
    } else {
      flyActive = 1;
      flySpeed = SPEEDHIGH;
    }
  }
  if(!yPressed && Controller1.ButtonY.pressing()){
    if(flySpeed == SPEEDLOW){
      flyActive = 0;
      flySpeed = 0;
    } else {
      flyActive = 1;
      flySpeed = SPEEDLOW;
    }
  }

  xPressed = Controller1.ButtonX.pressing();
  yPressed = Controller1.ButtonY.pressing();
  setSide(FlywheelGroup, flySpeed* flyActive);
}

void toggleIntake(){
  if(Controller1.ButtonL1.pressing()){
    setSide(frontIntake, 100);
  }else if (Controller1.ButtonL2.pressing()) {
    setSide(frontIntake, -100);
  }else {
    setSide(frontIntake, 0);
  }
}


double driverInput(double input){
  double out = 0;
  double x = fabs(input);

  if(x <= 30){
    out = (1/8.0)*(pow(x, 1.3));
  }else{
    out = 4 * exp(x / 28.5) - 1;
  }
  out = fabs(out) * sign(input);

  if(fabs(out) > 100){
    out = 100 * sign(out);
  }

  return out;
}

double lim(double n, double o, double dt){
  const double MaxPercentPerSecond = 450;
  double delta = n - o;
  if(fabs(delta) > MaxPercentPerSecond * dt){
    return o + MaxPercentPerSecond * dt * sign(delta);
  }
  return n;
}

void tankDriveStandard(){
  double left = driverInput(Controller1.Axis3.value());
  double right = driverInput(Controller1.Axis2.value());
  setSide(leftDriveMotorGroup, left);
  setSide(rightDriveMotorGroup, right);
}

void tankDriveExp(){
  static double lastTime = (Brain.timer(timeUnits::msec)-1) / 1000;
  static double lastLeft = 0;
  static double lastRight = 0;
  double deltaTime = Brain.timer(timeUnits::msec) / 1000 - lastTime;

  double left = driverInput(Controller1.Axis3.value());
  double right = driverInput(Controller1.Axis2.value());
  if(fabs(left) < 1){
    left = 0;
  }
  if(fabs(right) < 1){
    right = 0;
  }

  left = lim(left, lastLeft, deltaTime);
  right = lim(right, lastRight, deltaTime);

  setSide(leftDriveMotorGroup, left);
  setSide(rightDriveMotorGroup, right);
  lastLeft = left;
  lastRight = right;
  lastTime = Brain.timer(timeUnits::msec) / 1000;
}

const int DRIVER_TIME_IN_SECONDS = 75;
double DRIVER_START_TIME = 0;
bool ENDGAME_ARMED = false;

double getRemainingDriverTime(){
  return fmax(0, DRIVER_TIME_IN_SECONDS - (Brain.timer(timeUnits::sec) - DRIVER_START_TIME));
}

void fireEndgame(bool overide=false){
  if(ENDGAME_ARMED || overide){
    endGamePne.set(true);
  }
}

void triggerEndgame(bool btnPressing){
  static bool press = false;
  static bool reversed = false;

  double remainder = getRemainingDriverTime();

  if(btnPressing){
    if(!press){
      //if(remainder < 10){
        //if(remainder > 3){
          if(ENDGAME_ARMED){
            fireEndgame();
          }else{
            ENDGAME_ARMED = true;
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("ARMED ARMED");
            Controller1.rumble("....");
          }
        //}else{
          //fireEndgame(true);
        //}
      //}
    }
  }

  static bool triggered = false;
  if(remainder < 10 && !triggered){
    Controller1.rumble("----");
    triggered = true;
  }

  press = btnPressing;
}

double flipping = 0;

void usercontrol(void) {
  robot.stopMotors();
  rightDriveMotorGroup.setStopping(brakeType::coast);
  leftDriveMotorGroup.setStopping(brakeType::coast);

  DRIVER_START_TIME = Brain.timer(timeUnits::sec);
  while(true){
    tankDriveStandard();
    //tankDriveExp();

    setSide(Roller1, (Controller1.ButtonR1.pressing() - Controller1.ButtonR2.pressing())*100);
    setSide(Endgame, (Controller1.ButtonLeft.pressing() - Controller1.ButtonRight.pressing())*100);

    flywheelControl();

    toggleIntake();
    
    if(Controller1.ButtonB.pressing()){
      trigger.startRotateTo(95, rotationUnits::deg, 100, velocityUnits::pct);
      flipping = Brain.timer(msec);
    } else {
      if(Brain.timer(msec) - flipping > 100){
        trigger.startRotateTo(0, rotationUnits::deg, 100, velocityUnits::pct);
      }
    }
    
    triggerEndgame(Controller1.ButtonUp.pressing());
    updateControllerDisplay();
    wait(20, timeUnits::msec);
  }
}

#endif