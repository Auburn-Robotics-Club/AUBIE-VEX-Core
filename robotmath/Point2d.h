#pragma once
#include "base.h"

/**
 * Class for a point in 2-dimensional space.
*/
class Point2d{
  public:
    double x;
    double y;

    /**
     * Initialize a new Point2d object at (0, 0).
    */
    Point2d();

    /**
     * Initialize a new Point2d object at a custom location.
     * @param X The x-coordinate of the point.
     * @param Y The y-coordinate of the point.
    */
    Point2d(double X, double Y);
};

//Print operation (x, y)
std::ostream& operator << (std::ostream& os, Point2d p);

Point2d operator + (Point2d p, Vector2d v);
std::vector<Point2d> operator + (const std::vector<Point2d>& pList, const std::vector<Vector2d>& vList);
std::vector<Point2d> operator + (const std::vector<Vector2d>& vList, const std::vector<Point2d>& pList);

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, Vector2d v); //Offset each element in a list
std::vector<Point2d> operator * (const std::vector<Point2d>& pList, double scale); //Scale each element in a list realitive to first point
std::vector<Point2d> operator || (const std::vector<Point2d>& pList, double radiansCCW); //Rotate each element in list realitive to first point

/**
 * Calculate the midpoint between 2 points.
 * @param a The first point.
 * @param b The second point.
 * @return The midpoint of the 2 points.
*/
Point2d midpoint(Point2d a, Point2d b);

//positionSet
typedef struct {
    Point2d p; //Units
    double head; //Radians
} positionSet;