#ifndef __ROBOTMATH_H__
#define __ROBOTMATH_H__
#include "math.h"
#include <algorithm>
#include <vector>

/*
Name: robotmath.h
Written By: Carson Easterling

Defines abstract mathmatical concepts and algorithms relevent to robot control. Includes functionality for angles, PID algorithm, beizer curves, and vectors

*/

//Returns x if within min/max else return min/max
double clamp(double x, double min, double max);

//Same as clamp execept it treats lim as a min or max accoring to isMax
double clamp(double x, double lim, bool isMax);

//sign - returns 1 if input >= 0; -1 if input < 0
int sign(int x);
double sign(double x);

//Angles
const double M_2PI = 2*M_PI;
const double M_PI_180 = M_PI/180;
const double M_180_PI = 180/M_PI;

//Converts degrees to radians
double degToRad(double degrees);
//Converts radians to degrees
double radToDeg(double radians);

//Returns any angle between 0 - (360 || 2PI depending if input in in degrees or radians)
double normalizeAngle(double theta, bool inRadians=true);


//Takes two headings and gives you the smallest angle between them in radians; + if the target heading is counter clockwise of the current heading and - if clockwise
double shortestArcToTarget(double currentHeading, double targetHeading, bool inDeg=false);

//Takes two headings and returns the shortest angle between the current heading and a secent line through the circle that passes through the target heading
double shortestArcToLine(double currentHeading, double targetHeading, bool inDeg=false);

//--------------------------------------------------------------------------------------------------

class SMAFilter; //More stable; Simple Moving Average filter
class EWMAFilter; //Faster, less memory; Exponetinally weighted moving average filter - https://hackaday.com/2019/09/06/sensor-filters-for-coders/
//Moving median filter - Less suseptiable to shocks
//Kalman filter
class BasePIDController;
//SloshPIDController - Limits change of output
//
//For more reading: https://pidexplained.com/how-to-tune-a-pid-controller/

class SMAFilter{
protected:
  std::vector<double> data;

public:
  SMAFilter(int size);
  SMAFilter(int size, double value);

  void changeSize(double size, double value);

  void clear(double value=0);

  void add(double value);

  double getAvg();
};

class EWMAFilter{
protected:
  double k;
  double lastData;

public:
  EWMAFilter(double kIn);
  EWMAFilter(double kIn, double initalValue);

  void setK(double kIn);
  double getK();
  void setLastData(double dataIn);

  double getAvg(double dataIn);
};

class BasePIDController{
protected:
  double kP;
  double kI;
  double kD;

  double lastTime;
  double setValue;
  double lastError;


  double minOutput; //Limits output and setvalue
  double maxOutput;
public:
  //TODO
};


//--------------------------------------------------------------------------------------------------

//Point Class (Stores X and Y coordnates)
class Point2d{
  public:
    double x;
    double y;
    Point2d(double X, double Y);
};

//Vector Class (Repersents a Vector and allows vector operations to be performed)
class Vector2d{
  private:
    double deltaX; //Change in X repersented by the vector
    double deltaY; //Change in Y repersented of the vector
    
  public:
    //Define Vector by using dX and dY
    Vector2d(double delta_x, double delta_y);
    
    //Define Vector using start and end points
    Vector2d(Point2d start, Point2d end);

    //Define Vector by using a magnitiude at a given heading
    Vector2d(double magnitude, double theta, bool inDegrees);

    //Returns dX
    double getX();

    //Returns dY
    double getY();

    //Returns the magnitude of the vector
    double getMagnitude();

    //Returns the dot product of this vector and another (The product of both vector's magnitude, + if in same direction, 0 if perendicular, - if facing opposite directions)
    double dot(Vector2d otherV);

    //Returns the angle between this vector and another realitive to this vector in radians (+ is CCW; - is CW)
    double getAngle(Vector2d vecB);

    //Returns a new Vector based on a scaling factor of this vector
    Vector2d scale(double s);

    //Returns a new vector repersenting the unit vector of this vector
    Vector2d getUnitVector();

    //Returns a new vector that repersents this vector rotated by theta (+ is CCW; - is CW)
    Vector2d getRotatedVector(double theta, bool inDegrees=false);

    //Returns a new vector that repersents this vector projected onto another vector (The component of this vector in the same direction as the other vector)
    Vector2d project(Vector2d vec);
    
    //Adds two vectors using + (this + otherVector)
    Vector2d operator + (const Vector2d& other);

    //Subtracts two vectors using - (this - otherVector)
    Vector2d operator - (const Vector2d& other);

    //Returns a new vector scaled by factor (this * constInt)
    Vector2d operator * (int scalar);

    //Returns a new vector scaled by factor (this * constDouble)
    Vector2d operator * (double scalar);

    //Returns a new point that results from adding a vector from a point (this + point)
    Point2d operator + (Point2d &p);
    Point2d operator - (Point2d &p);
};

#endif