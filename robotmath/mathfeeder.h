#ifndef MATHFEEDER_H
#define MATHFEEDER_H

#define _USE_MATH_DEFINES
#include "math.h"

//Constants
const double M_2PI = 2*M_PI;
const double M_PI_180 = M_PI/180;
const double M_180_PI = 180/M_PI;
const double MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA = 0.001; //0.5 deg in radians = 0.008 thus 0.001 should be accurate enough given sensor capability

struct PIDValues {
    double P;
    double I;
    double D;
};

typedef struct {
    Point2d p; //Units
    double head; //Radians
} positionSet;

//positionSet
std::ostream& operator << (std::ostream& os, positionSet p);
bool operator==(const positionSet& a, const positionSet& b);
bool operator!=(const positionSet& a, const positionSet& b);

#endif