#include "vex.h"
#include <cmath>

using namespace vex;

void setM(motor m, double value, velocityUnits units = velocityUnits::pct) {
  if(fabs(value) < 0.1) {m.stop(); return;}
  m.spin(fwd, value, units);
}

void setM(motor_group m, double value, velocityUnits units = velocityUnits::pct) {
  if(fabs(value) < 0.1) {m.stop(); return;}
  m.spin(fwd, value, units);
}