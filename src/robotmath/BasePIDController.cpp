#include "../../robotmath/robotmath.h"

BasePIDController::BasePIDController(PIDValues pid_values, double init_target, double min_output, double max_output){
    k = pid_values;
    setTarget(init_target);
    minOutput = min_output;
    maxOutput = max_output;
}

void BasePIDController::setTarget(double targetSpeed, double initSetValue){
  target = targetSpeed;
  setValue = initSetValue;
  lastError = 0;
  initalized = false;
}

double BasePIDController::update(double currentValue, double deltaTime){
  double error = target - currentValue;
  if(!initalized){lastError = error; initalized=true;}

  double deltaError = error - lastError;
  lastError = error;

  setValue = clamp(setValue + k.I * error * deltaTime, minOutput, maxOutput);
  
  output = clamp(
            (k.P * error) + (setValue) + (k.D * deltaError / deltaTime), 
            minOutput, maxOutput);
  return output;
}