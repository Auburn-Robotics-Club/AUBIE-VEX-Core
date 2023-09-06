#ifndef VECTOR_2D_H
#define VECTOR_2D_H

#include <sstream>
#include <vector>
#include "Point2d.h"

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

Vector2d operator - (const Point2d endPoint, const Point2d startPoint);
std::ostream& operator << (std::ostream& os, Vector2d v);
std::vector<Vector2d> operator + (std::vector<Vector2d>& vList, Vector2d v); //Offset each element in a list
std::vector<Vector2d> operator * (std::vector<Vector2d>& vList, double scale); //Scale each element in a list
std::vector<Vector2d> operator || (std::vector<Vector2d>& vList, double radiansCCW); //Rotate each element in list

#endif