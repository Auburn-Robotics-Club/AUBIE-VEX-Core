#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#include <algorithm>
#include <sstream>
#include "mathfeeder.h"
#include "SMAFilter.h"
#include "EWMAFilter.h"
#include "BasePIDController.h"
#include "Point2d.h"
#include "Vector2d.h"
#include "Path.h"

/*
Name: robotmath.h
Written By: Carson Easterling

Defines abstract mathmatical concepts and algorithms relevent to robot control. Includes functionality for angles, PID algorithm, beizer curves, and vectors

*/

//Filters
//--------------------------------------------------------------------------------------------------
//Moving median filter - Less suseptiable to shocks
//Kalman filter
//Time based PID controller
//SloshPIDController - Limits change of output
//BamBamController
//For more reading: https://pidexplained.com/how-to-tune-a-pid-controller/

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

//Returns a point along a defined bezier curve at t (0-1)
Point2d bezierFormula(Point2d initPoint, Point2d finalPoint, Point2d C1, double t);

//Bezier Path using control points
//Recommended to use pass by reference functions rather than returns to save memory
std::vector<Point2d> generateCurve(Point2d start, Point2d end, Point2d c1, bool includeC1 = false, int steps = 10);
std::vector<Point2d> generateCurve(Point2d start, Vector2d end, Vector2d v1, bool includeC1 = false, int steps = 10);
void generateCurve(std::vector<Point2d>& points, Point2d start, Point2d end, Point2d c1, bool includeC1 = false, int steps = 10);
void generateCurve(std::vector<Point2d>& points, Point2d start, Vector2d end, Vector2d v1, bool includeC1 = false, int steps = 10);

std::vector<Point2d> generateCurve(Point2d start, Point2d end, std::vector<Point2d>& controlPoints, bool includeC1 = false, int steps = 10);
std::vector<Point2d> generateCurve(Point2d start, Vector2d end, std::vector<Vector2d>& controlVectors, bool includeC1 = false, int steps = 10);
void generateCurve(std::vector<Point2d>& points, Point2d start, Point2d end, std::vector<Point2d>& controlPoints, bool includeC1 = false, int steps = 10);
void generateCurve(std::vector<Point2d>& points, Point2d start, Vector2d end, std::vector<Vector2d>& controlVectors, bool includeC1 = false, int steps = 10);

std::vector<positionSet> curveHeadings(std::vector<Point2d>& points);
void curveHeadings(std::vector<positionSet>& posSet, std::vector<Point2d>& points);

std::ostream& operator << (std::ostream& os, positionSet p);
bool operator==(const positionSet& a, const positionSet& b);
bool operator!=(const positionSet& a, const positionSet& b);
positionSet predictLinear(positionSet start, Vector2d vel, double w, double t);
//Tank drive assumed
positionSet predictWithConstantTurning(positionSet start, Vector2d vel, double w, double t);

double calculateRadiusOfCurvature(Point2d start, Point2d middle, Point2d end);

#endif