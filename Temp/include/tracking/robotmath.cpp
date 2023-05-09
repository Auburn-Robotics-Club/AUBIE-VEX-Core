#include "tracking/robotmath.h"

int clamp(int x, int min, int max){
  if(x < min){return min;}
  if(x > max){return max;}
  return x;
}

double fclamp(double x, double min, double max){
  if(x < min){return min;}
  if(x > max){return max;}
  return x;
}

//sign - returns 1 if input >= 0; -1 if input < 0
int sign(int x){
  if(x < 0){
    return -1;
  }
  return 1;
}

double sign(double x){
  if(x < 0){
    return -1;
  }
  return 1;
}

// Converts degrees to radians
double degToRad(double degrees) { return degrees * M_PI_180; }

// Converts radians to degrees
double radToDeg(double radians) { return radians * M_180_PI; }

// Returns any angle between 0 - (360 || 2PI depending if input in in degrees or
// radians)
double normalizeAngle(double theta, bool inRadians) {
  double toss;
  if (inRadians) {
    double r = (M_2PI)*modf(theta / (M_2PI), &toss);
    if (r < 0) {
      r += (M_2PI);
    }
    return r;
  } else {
    double r = (360) * modf(theta / (360), &toss);
    if (r < 0) {
      r += (360);
    }
    return r;
  }
}

// Takes two headings and gives you the smallest angle between them in radians;
// + if the target heading is counter clockwise of the current heading and - if
// clockwise
double shortestArcToTarget(double currentHeading, double targetHeading,
                           bool inDeg) {
  if (inDeg) {
    currentHeading = degToRad(currentHeading);
    targetHeading = degToRad(targetHeading);
  }
  currentHeading = normalizeAngle(currentHeading);
  targetHeading = normalizeAngle(targetHeading);
  double deltaTheta = targetHeading - currentHeading;
  if (fabs(deltaTheta) > M_PI) {
    if (deltaTheta < 0) {
      // If E is CW then faster path is CCW
      deltaTheta = M_2PI + deltaTheta;
    } else {
      // If E is CCW; faster path is CW
      deltaTheta = deltaTheta - M_2PI;
    }
  }
  return deltaTheta;
}

// Takes two headings and returns the shortest angle between the current heading
// and a secent line through the circle that passes through the target heading
double shortestArcToLine(double currentHeading, double targetHeading, bool inDeg) {
  if (inDeg) {
    currentHeading = degToRad(currentHeading);
    targetHeading = degToRad(targetHeading);
  }
  currentHeading = normalizeAngle(currentHeading);
  targetHeading = normalizeAngle(targetHeading);
  double deltaTheta = targetHeading - currentHeading;
  if (fabs(deltaTheta) <= M_PI_2) {
    return shortestArcToTarget(currentHeading, targetHeading);
  } else {
    return shortestArcToTarget(currentHeading, targetHeading + M_PI);
  }
}

//--------------------------------------------------------------------------------------------------

SMAFilter::SMAFilter(int size) { 
  changeSize(size, 0); 
}
SMAFilter::SMAFilter(int size, double value) { 
  data.resize(size, value); 
}

void SMAFilter::changeSize(double size, double value) {
  if (size < data.size()) {
    rotate(data.begin(), data.begin() + data.size() - size, data.end());
  }
  data.resize(size, value);
}

void SMAFilter::clear(double value) {
  for (int i = 0; i < data.size(); i++) {
    data[i] = value;
  }
}

void SMAFilter::add(double value) {
  rotate(data.begin(), data.begin() + 1, data.end()); // Shift Left
  data.back() = value; // Replace last element with new data
}

double SMAFilter::getAvg() {
  double sum = 0;
  int s = data.size();
  for (int i = 0; i < s; i++) {
    sum += data[i];
  }
  return sum / s;
}

EWMAFilter::EWMAFilter(double kIn){
  setK(kIn);
}

EWMAFilter::EWMAFilter(double kIn, double initalValue){
  setK(kIn);
  lastData = initalValue;
}

void EWMAFilter::setK(double kIn){
  k = clamp(kIn, 0, 1);
}

double EWMAFilter::getK(){
  return k;
}

void EWMAFilter::setLastData(double dataIn){
  lastData = dataIn;
}

double EWMAFilter::getAvg(double dataIn){
  double r = (1-k)*lastData + k*dataIn;
  lastData = dataIn;
  return r;
}

void BasePIDController::setTarget(double targetSpeed, double initSetValue){
  target = targetSpeed;
  setValue = initSetValue;
  lastError = 0;
  initalized = false;
}

double BasePIDController::update(double currentValue, double deltaTime){
  double error = target - currentValue;
  if(!initalized){lastError = error; initalized=true;}

  double deltaError = error - lastError;
  lastError = error;

  setValue = clamp(setValue + kI * error * deltaTime, minOutput, maxOutput);
  
  output = clamp(
            (kP * error) + (setValue) + (kD * deltaError / deltaTime), 
            minOutput, maxOutput);
  return output;
}

//--------------------------------------------------------------------------------------------------

// Point Class (Stores X and Y coordnates)
Point2d::Point2d(double X, double Y) {
  x = X;
  y = Y;
}

// Vector Class (Repersents a Vector and allows vector operations to be performed)

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

// Returns a new point that results from adding a vector from a point (this +
// point)
Point2d Vector2d::operator+(Point2d &p) {
  return Point2d(p.x + getX(), p.y + getY());
}
Point2d Vector2d::operator-(Point2d &p) {
  return Point2d(p.x - getX(), p.y - getY());
}