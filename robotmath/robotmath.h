#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#include "Point2d.h"
#include "Vector2d.h"
#include "Path.h"
#include "BasePIDController.h"
#include "EWMAFilter.h"
#include "SMAFilter.h"

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

//Returns a point along a defined bezier curve at t (0-1)
Point2d bezierFormula(Point2d initPoint, Point2d finalPoint, Point2d C1, double t);

//Bezier Path using control points
//Recommended to use pass by reference functions rather than returns to save memory
Path generateCurve(Point2d start, Point2d end, Point2d c1, bool includeC1 = false, int steps = 10);
Path generateCurve(Point2d start, Vector2d end, Vector2d v1, bool includeC1 = false, int steps = 10);

Path generateCurve(Point2d start, Point2d end, std::vector<Point2d>& controlPoints, bool includeC1 = false, int steps = 10);
Path generateCurve(Point2d start, Vector2d end, std::vector<Vector2d>& controlVectors, bool includeC1 = false, int steps = 10);

void curveHeadings(Path points);

std::ostream& operator << (std::ostream& os, positionSet p);
bool operator==(const positionSet& a, const positionSet& b);
bool operator!=(const positionSet& a, const positionSet& b);
positionSet predictLinear(positionSet start, Vector2d vel, double w, double t);
//Tank drive assumed
positionSet predictWithConstantTurning(positionSet start, Vector2d vel, double w, double t);

double calculateRadiusOfCurvature(Point2d start, Point2d middle, Point2d end);

#endif