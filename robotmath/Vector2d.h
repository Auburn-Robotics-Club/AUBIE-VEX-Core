#pragma once
#include "base.h"

/**
 * Class representing a vector with 2 dimensions.
*/
class Vector2d{
  private:
    double deltaX; //Change in X repersented by the vector
    double deltaY; //Change in Y repersented of the vector
    
  public:
    /**
     * Initialize a new Vector2d object with components <1, 1>.
     */
    Vector2d();

    /**
     * Initialize a new Vector2d object with custom delta-x and delta-y components.
     * 
     * @param delta_x The x component of the vector.
     * @param delta_y The y component of the vector.
     */
    Vector2d(double delta_x, double delta_y);
    
    /**
     * Initialize a new Vector2d object with start and end points.
     * 
     * @param start A Point2d object representing the start point.
     * @param end A Point2d object representing the end point.
     */
    Vector2d(Point2d start, Point2d end);

    /**
     * Initialize a new Vector2d object with a magnitude and direction.
     * 
     * @param magnitude The magnitude of the vector.
     * @param theta The angle of the vector relative to the x-axis.
     * @param inDegrees A boolean representing whether the angle is in degrees.
     */
    Vector2d(double magnitude, double theta, bool inDegrees);

    /**
     * Retrieve the x-component of the vector.
     * 
     * @return The value of the x-component.
     */
    double getX();

    /**
     * Retrieve the y-component of the vector.
     * 
     * @return The value of the y-component.
     */
    double getY();

    /**
     * Retrieve the magnitude of the vector.
     * 
     * @return The value of the magnitude.
     */
    double getMagnitude();

    /**
     * Returns the dot product of this vector and another vector.
     * (The product of both vector's magnitude, + if in same direction, 0 if perendicular, - if facing opposite directions)
     * 
     * @param otherV The other vector to take the dot product with.
     * 
     * @return The value of the dot product.
     */
    double dot(Vector2d otherV);

    /**
     * Returns the cross product of this vector and another vector.
     * 
     * @param otherV The other vector to take the cross product with.
     * 
     * @return The value of the cross product.
     */
    double cross(Vector2d otherV);

    /**
     * Returns the angle between this vector and another vector.
     * 
     * @param vecB The other vector to find the angle with.
     * 
     * @return The value of the angle in radians (+ is CCW; - is CW).
     */
    double getAngle(Vector2d vecB);

    /**
     * Returns a scaled version of this vector.
     * 
     * @param s The scale to use.
     * 
     * @return A new Vector2d object representing the scaled vector.
     */
    Vector2d scale(double s);

    /**
     * Returns the unit vector of this vector.
     * 
     * @return A new Vector2d object representing the unit vector.
     */
    Vector2d getUnitVector();

    /**
     * Returns a rotated version of this vector.
     * 
     * @param theta The angle to rotate this vector by (+ is CCW; - is CW).
     * @param inDegrees A boolean of whether theta is in degrees (false by default).
     * 
     * @return A new Vector2d object representing the rotated vector.
     */
    Vector2d getRotatedVector(double theta, bool inDegrees=false);

    /**
     * Returns this vector projected onto another vector.
     * (The component of this vector in the same direction as the other vector).
     * 
     * @param vec The vector to project onto.
     * 
     * @return A Vector2d object representing the projection.
     */
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