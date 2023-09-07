#include "robotmath.h"

// Define Vector
Vector2d::Vector2d() {
    deltaX = 1;
    deltaY = 1;
}

// Define Vector by using dX and dY
Vector2d::Vector2d(double delta_x, double delta_y) {
  deltaX = delta_x;
  deltaY = delta_y;
}

// Define Vector using start and end points
Vector2d::Vector2d(Point2d start, Point2d end) {
  deltaX = end.x - start.x;
  deltaY = end.y - start.y;
}

// Define Vector by using a magnitiude at a given heading
Vector2d::Vector2d(double magnitude, double theta, bool inDegrees) {
  if (inDegrees) {
    theta = degToRad(theta);
  }
  deltaX = magnitude * cos(theta);
  deltaY = magnitude * sin(theta);
}

// Returns dX
double Vector2d::getX() { return deltaX; }

// Returns dY
double Vector2d::getY() { return deltaY; }

// Returns the magnitude of the vector
double Vector2d::getMagnitude() {
  return sqrt(deltaX * deltaX + deltaY * deltaY);
}

// Returns the dot product of this vector and another (The product of both vector's magnitude, + if in same direction, 0 if perendicular, - if facing opposite directions)
double Vector2d::dot(Vector2d otherV) {
  return deltaX * otherV.deltaX + deltaY * otherV.deltaY;
}

// Returns the cross product of this vector and another
double Vector2d::cross(Vector2d otherV) {
    return deltaX * otherV.deltaY - deltaY * otherV.deltaX;
}

// Returns the angle between this vector and another realitive to this vector in radians (+ is CCW; - is CW)
double Vector2d::getAngle(Vector2d vecB) {
  if ((getMagnitude() == 0) || (vecB.getMagnitude() == 0)) {
    return 0; // Its bound between 0-PI so 2PI is error indication
  }
  double theta = acos(dot(vecB) / (getMagnitude() * vecB.getMagnitude()));
  double dotZ = deltaX * -vecB.deltaY + deltaY * vecB.deltaX;
  if (dotZ > 0) {
    // B right of A
    return -theta;
  } else if (dotZ < 0) {
    // B left of A
    return theta;
  } else {
    // Paraell
    return theta;
  }
}

// Returns a new Vector based on a scaling factor of this vector
Vector2d Vector2d::scale(double s) { return Vector2d(deltaX * s, deltaY * s); }

// Returns a new vector repersenting the unit vector of this vector
Vector2d Vector2d::getUnitVector() {
  if (getMagnitude() == 0) {
    return Vector2d(1, 0);
  }
  return Vector2d(deltaX / getMagnitude(), deltaY / getMagnitude());
}

// Returns a new vector that repersents this vector rotated by theta (+ is
// CCW; - is CW)
Vector2d Vector2d::getRotatedVector(double theta, bool inDegrees) {
  if (inDegrees) {
    theta = degToRad(theta);
  }
  return Vector2d(deltaX * cos(theta) - deltaY * sin(theta),
                  deltaX * sin(theta) + deltaY * cos(theta));
}

// Returns a new vector that repersents this vector projected onto another
// vector (The component of this vector in the same direction as the other
// vector)
Vector2d Vector2d::project(Vector2d vec) {
  // Projects this on to vec
  if (vec.getMagnitude() == 0) {
    return Vector2d(0, 0);
  }
  return vec.scale(dot(vec) / (vec.getMagnitude() * vec.getMagnitude()));
}

// Adds two vectors using + (this + otherVector)
Vector2d Vector2d::operator+(const Vector2d &other) {
  return Vector2d(deltaX + other.deltaX, deltaY + other.deltaY);
}

// Subtracts two vectors using - (this - otherVector)
Vector2d Vector2d::operator-(const Vector2d &other) {
  return Vector2d(deltaX - other.deltaX, deltaY - other.deltaY);
}

// Returns a new vector scaled by factor (this * constInt)
Vector2d Vector2d::operator*(int scalar) {
  return Vector2d(deltaX * scalar, deltaY * scalar);
}

// Returns a new vector scaled by factor (this * constDouble)
Vector2d Vector2d::operator*(double scalar) {
  return Vector2d(deltaX * scalar, deltaY * scalar);
}

//Dot Product
double Vector2d::operator * (Vector2d v) {
    return dot(v);
};

//Cross Product
double Vector2d::operator || (Vector2d v) {
    return cross(v);
};

//Rotate
Vector2d Vector2d::operator|| (double radiansCCW) {
    return getRotatedVector(radiansCCW);
};

Point2d Vector2d::operator+(Point2d& p) {
    return Point2d(p.x + getX(), p.y + getY());
}

Point2d Vector2d::operator-(Point2d& p) {
    return Point2d(p.x - getX(), p.y - getY());
}

std::ostream& operator << (std::ostream& os, Vector2d v){
  os << "<" << v.getX() << ", " << v.getY() << ">";
  return os;
}

std::vector<Vector2d> operator + (std::vector<Vector2d>& vList, Vector2d v) {
    std::vector<Vector2d> result;
    for (int i = 0; i < vList.size(); i++) {
        result.push_back(vList[i] + v);
    }
    return result;
};

std::vector<Vector2d> operator * (std::vector<Vector2d>& vList, double scale) {
    std::vector<Vector2d> result;
    for (int i = 0; i < vList.size(); i++) {
        result.push_back(vList[i].scale(scale));
    }
    return result;
};

std::vector<Vector2d> operator || (std::vector<Vector2d>& vList, double radiansCCW) {
    std::vector<Vector2d> result;
    for (int i = 0; i < vList.size(); i++) {
        result.push_back(vList[i].getRotatedVector(radiansCCW));
    }
    return result;
};

Vector2d operator - (const Point2d endPoint, const Point2d startPoint) {
    return Vector2d(startPoint, endPoint);
};