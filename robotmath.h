#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#define _USE_MATH_DEFINES
#include "math.h"
#include <algorithm>
#include <vector>
#include <sstream>

/*
Name: robotmath.h
Written By: Carson Easterling

Defines abstract mathmatical concepts and algorithms relevent to robot control. Includes functionality for angles, PID algorithm, beizer curves, and vectors

*/

//Constants
//--------------------------------------------------------------------------------------------------
const double M_2PI = 2*M_PI;
const double M_PI_180 = M_PI/180;
const double M_180_PI = 180/M_PI;
const double MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA = 0.001; //0.5 deg in radians = 0.008 thus 0.001 should be accurate enough given sensor capability

//Filters
//--------------------------------------------------------------------------------------------------

//Moving median filter - Less suseptiable to shocks
//Kalman filter
//SloshPIDController - Limits change of output
//BamBamController
//For more reading: https://pidexplained.com/how-to-tune-a-pid-controller/

//SMAFilter 
//More stable; Simple Moving Average filter
//--------------------------------------------------------------------------------------------------
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

//EWMAFilter 
//Faster, less memory; Exponetinally weighted moving average filter - https://hackaday.com/2019/09/06/sensor-filters-for-coders/
//--------------------------------------------------------------------------------------------------
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

//BasePIDController
//--------------------------------------------------------------------------------------------------
class BasePIDController{
protected:
  double kP;
  double kI;
  double kD;

  double target;
  double error;

  double setValue;

  bool initalized;
  double lastError;
  
  double minOutput; //Limits output and setvalue
  double maxOutput;

  double output;
public:
  void setTarget(double targetValue, double initSetValue=0);
  double update(double currentValue, double deltaTime);
};


//Point2d
//--------------------------------------------------------------------------------------------------
class Vector2d;
class Point2d;

class Point2d{
  public:
    double x;
    double y;
    Point2d();
    Point2d(double X, double Y);
};

//Print operation (x, y)
std::ostream& operator << (std::ostream& os, Point2d p);

//Returns midpoint
Point2d midpoint(Point2d a, Point2d b);

Vector2d operator - (const Point2d endPoint, const Point2d startPoint);
Point2d operator + (Point2d p, Vector2d v);
std::vector<Point2d> operator + (const std::vector<Point2d>& pList, const std::vector<Vector2d>& vList);
std::vector<Point2d> operator + (const std::vector<Vector2d>& vList, const std::vector<Point2d>& pList);

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, Vector2d v); //Offset each element in a list
std::vector<Point2d> operator * (const std::vector<Point2d>& pList, double scale); //Scale each element in a list realitive to first point
std::vector<Point2d> operator || (const std::vector<Point2d>& pList, double radiansCCW); //Rotate each element in list realitive to first point


//positionSet
//--------------------------------------------------------------------------------------------------
typedef struct {
    Point2d p; //Units
    double head; //Radians
} positionSet;

std::ostream& operator << (std::ostream& os, positionSet p);
bool operator==(const positionSet& a, const positionSet& b);
bool operator!=(const positionSet& a, const positionSet& b);

//Vector2d
//--------------------------------------------------------------------------------------------------
class Vector2d{
  private:
    double deltaX; //Change in X repersented by the vector
    double deltaY; //Change in Y repersented of the vector
    
  public:
    //Define Vector
    Vector2d();

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

    //Returns the cross product of this vector and another
    double cross(Vector2d otherV);

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

    //Dot Product
    double operator * (Vector2d v);

    //Cross Product
    double operator || (Vector2d v);

    //Rotate
    Vector2d operator || (double radiansCCW);

    //Returns a new point that results from adding a vector from a point (this + point)
    Point2d operator + (Point2d &p);
    Point2d operator - (Point2d &p);
};

std::ostream& operator << (std::ostream& os, Vector2d v);
std::vector<Vector2d> operator + (std::vector<Vector2d>& vList, Vector2d v); //Offset each element in a list
std::vector<Vector2d> operator * (std::vector<Vector2d>& vList, double scale); //Scale each element in a list
std::vector<Vector2d> operator || (std::vector<Vector2d>& vList, double radiansCCW); //Rotate each element in list

//Misc Functions
//--------------------------------------------------------------------------------------------------

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

positionSet predictLinear(positionSet start, Vector2d vel, double w, double t);
//Tank drive assumed
positionSet predictWithConstantTurning(positionSet start, Vector2d vel, double w, double t);

double calculateRadiusOfCurvature(Point2d start, Point2d middle, Point2d end);

//Path
//--------------------------------------------------------------------------------------------------
class Path {
protected:
    int internalIndex = 0;
    std::vector<positionSet> points;

public:
    Path();
    Path(std::vector<positionSet> &pointsIn);

    std::vector<positionSet>& getList();
    void setIndex(int i = 0);
    int size();
    positionSet next();
    bool hasNext();
    int index();
    positionSet get(int i);
    void drop(int x);
    void clear();

    void addPointset(positionSet Point);
    void addPointset(Point2d p, double head, bool inDeg = true);

    double arclength(int start = 0, int end = -1);
    double arclengthFromIndexTo(int end = -1);

    void subpath(Path& pathIn, int start, int end);
    Path subpath(int start, int end);

    //Append
    positionSet operator [] (int index);
    Path& operator + (positionSet& p);
    Path& operator + (Path& p);
};

std::ostream& operator << (std::ostream& os, Path& p);

#endif