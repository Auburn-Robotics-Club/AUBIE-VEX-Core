#pragma once
#include <algorithm>
#include <sstream>
#include <vector>

#define _USE_MATH_DEFINES
#include "math.h"

//Constants
const double M_2PI = 2*M_PI;
const double M_PI_180 = M_PI/180;
const double M_180_PI = 180/M_PI;
const double MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA = 0.001; //0.5 deg in radians = 0.008 thus 0.001 should be accurate enough given sensor capability

class Point2d;
//positionSet defined as part of Point2d
class Vector2d;

struct PIDValues {
    double P;
    double I;
    double D;
};

//Misc Functions
//--------------------------------------------------------------------------------------------------

//Returns minimum number
int min(int a, int b);
double fmin(double a, double b);

//Returns maximum number
int max(int a, int b);
double fmax(double a, double b);

//Returns x if within min/max else return min/max
int clamp(int x, int min, int max);
double fclamp(double x, double min, double max);

//sign - returns 1 if input >= 0; -1 if input < 0
int sign(int x);
double sign(double x);

//Angles

//Converts degrees to radians
double degToRad(double degrees);
//Converts radians to degrees
double radToDeg(double radians);

//Returns any angle between 0 - (360 || 2PI depending if input in in degrees or radians)
double normalizeAngle(double theta, bool inRadians = true);


//Takes two headings and gives you the smallest angle between them in radians; + if the target heading is counter clockwise of the current heading and - if clockwise
double shortestArcToTarget(double currentHeading, double targetHeading, bool inDeg = false);

//Takes two headings and returns the shortest angle between the current heading and a secent line through the circle that passes through the target heading
double shortestArcToLine(double currentHeading, double targetHeading, bool inDeg = false);