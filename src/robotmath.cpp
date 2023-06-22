#include "../robotmath.h"

//Filters
//--------------------------------------------------------------------------------------------------

//SMAFilter
//-------------------------------------------------------------------------------------------------
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

//EWMAFilter
//--------------------------------------------------------------------------------------------------
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

//BasePIDController
//--------------------------------------------------------------------------------------------------
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


//Point2d
//--------------------------------------------------------------------------------------------------
Point2d::Point2d() {
    x = 0;
    y = 0;
}

Point2d::Point2d(double X, double Y) {
  x = X;
  y = Y;
}

std::ostream& operator << (std::ostream& os, Point2d p){
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

//Calculates midpoint of 2 points
Point2d midpoint(Point2d a, Point2d b) {
    return Point2d((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
}

Vector2d operator - (const Point2d endPoint, const Point2d startPoint) {
    return Vector2d(startPoint, endPoint);
};

Point2d operator + (Point2d p, Vector2d v) {
    return v + p;
};

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, const std::vector<Vector2d>& vList) {
    std::vector<Point2d> result;
    if (pList.size() >= vList.size()) {
        for (int i = 0; i < vList.size(); i++) {
            result.push_back(pList[i] + vList[i]);
        }
    } else {
        for (int i = 0; i < pList.size(); i++) {
            result.push_back(pList[i] + vList[i]);
        }
    }
    return result;
};

std::vector<Point2d> operator + (const std::vector<Vector2d>& vList, const std::vector<Point2d>& pList) {
    return pList + vList;
};

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, Vector2d v) {
    std::vector<Point2d> result;
    for (int i = 0; i < pList.size(); i++) {
        result.push_back(pList[i] + v);
    }
    return result;
};

std::vector<Point2d> operator * (const std::vector<Point2d>& pList, double scale) {
    std::vector<Point2d> result;
    for (int i = 1; i < pList.size(); i++) {
        result.push_back(pList[0] + (Vector2d(pList[0], pList[i]).scale(scale)));
    }
    return result;
};

std::vector<Point2d> operator || (const std::vector<Point2d>& pList, double radiansCCW) {
    std::vector<Point2d> result;
    if (pList.size() > 0) {
        result.push_back(pList[0]);
        for (int i = 1; i < pList.size(); i++) {
            result.push_back(pList[0] + (Vector2d(pList[0], pList[i]).getRotatedVector(radiansCCW)));
        }
    }
    return result;
};


//positionSet
//--------------------------------------------------------------------------------------------------
std::ostream& operator << (std::ostream& os, positionSet p) {
    os << "(" << p.p.x << ", " << p.p.y << ", " << p.head << ")";
    return os;
}

bool operator==(const positionSet& a, const positionSet& b) {
    if ((fabs(a.p.x - b.p.x) < MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA) && (fabs(a.p.y - b.p.y) < MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA) && (fabs(a.head - b.head) < MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA)) {
        return true;
    }
    else {
        return false;
    }
}

bool operator!=(const positionSet& a, const positionSet& b) {
    return !(a == b);
}

//Vector2d
//--------------------------------------------------------------------------------------------------

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


//Misc Functions
//--------------------------------------------------------------------------------------------------
int clamp(int x, int min, int max) {
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
}

double fclamp(double x, double min, double max) {
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
}

//sign - returns 1 if input >= 0; -1 if input < 0
int sign(int x) {
    if (x < 0) {
        return -1;
    }
    return 1;
}

double sign(double x) {
    if (x < 0) {
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
    }
    else {
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
        }
        else {
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
    }
    else {
        return shortestArcToTarget(currentHeading, targetHeading + M_PI);
    }
}

Point2d bezierFormula(Point2d initPoint, Point2d finalPoint, Point2d C1, double t) {
    double inverseT = (1 - t);
    double x = std::pow(inverseT, 2) * initPoint.x + inverseT * 2 * t * C1.x + std::pow(t, 2) * finalPoint.x;
    double y = std::pow(inverseT, 2) * initPoint.y + inverseT * 2 * t * C1.y + std::pow(t, 2) * finalPoint.y;
    return Point2d(x, y);
}

std::vector<Point2d> generateCurve(Point2d start, Point2d end, Point2d c1, bool includeC1, int steps) {
    if (includeC1) {
        c1 = Point2d(c1.x * 2 - (start.x + end.x) / 2, c1.y * 2 - (start.y + end.y) / 2);
    }
    std::vector<Point2d> results;
    double tStep = 1.0 / steps;
    double headTarget = 0;
    for (double t = 0; t <= 1; t = t + tStep) {
        results.push_back(bezierFormula(start, end, c1, t));
    }
    return results;
};

std::vector<Point2d> generateCurve(Point2d start, Vector2d end, Vector2d v1, bool includeC1, int steps) {
    return generateCurve(start, end + start, v1 + start, includeC1, steps);
};

void generateCurve(std::vector<Point2d>& points, Point2d start, Point2d end, Point2d c1, bool includeC1, int steps) {
    if (includeC1) {
        c1 = Point2d(c1.x * 2 - (start.x + end.x) / 2, c1.y * 2 - (start.y + end.y) / 2);
    }
    double tStep = 1.0 / steps;
    double headTarget = 0;
    for (double t = 0; t <= 1; t = t + tStep) {
        points.push_back(bezierFormula(start, end, c1, t));
    }
};

void generateCurve(std::vector<Point2d>& points, Point2d start, Vector2d end, Vector2d v1, bool includeC1, int steps) {
    return generateCurve(points, start, end + start, v1 + start, includeC1, steps);
};

std::vector<Point2d> generateCurve(Point2d start, Point2d end, std::vector<Point2d>& controlPoints, bool includeC1, int steps) {
    std::vector<Point2d> results;
    if (controlPoints.size() > 0) {
        if (controlPoints.size() > 1) {
            std::vector<Point2d> endpoints;
            endpoints.push_back(start);
            for (int i = 0; i < controlPoints.size() - 1; i++) {
                endpoints.push_back(midpoint(controlPoints[i], controlPoints[i + 1]));
            }
            endpoints.push_back(end);

            double tStep = 1.0 / steps;
            double headTarget = 0;
            for (int i = 0; i < endpoints.size() - 1; i++) {
                Point2d A = endpoints[i];
                Point2d B = endpoints[i + 1];
                Point2d c1 = controlPoints[i];
                if (includeC1) {
                    c1 = Point2d(c1.x * 2 - (A.x + B.x) / 2, c1.y * 2 - (A.y + B.y) / 2);
                }
                for (double t = 0; t <= 1; t = t + tStep) {
                    results.push_back(bezierFormula(A, B, c1, t));
                }
            }
        }
        else {
            return generateCurve(start, end, controlPoints[0], includeC1, steps);
        }
    }
    return results;
};

std::vector<Point2d> generateCurve(Point2d start, Vector2d end, std::vector<Vector2d>& controlVectors, bool includeC1, int steps) {
    std::vector<Point2d> results;
    if (controlVectors.size() > 0) {
        if (controlVectors.size() > 1) {
            std::vector<Point2d> endpoints;
            endpoints.push_back(start);
            for (int i = 0; i < controlVectors.size() - 1; i++) {
                endpoints.push_back(midpoint(controlVectors[i] + start, controlVectors[i + 1] + start));
            }
            endpoints.push_back(end + start);

            double tStep = 1.0 / steps;
            double headTarget = 0;
            for (int i = 0; i < endpoints.size() - 1; i++) {
                Point2d A = endpoints[i];
                Point2d B = endpoints[i + 1];
                Point2d c1 = controlVectors[i] + start;
                if (includeC1) {
                    c1 = Point2d(c1.x * 2 - (A.x + B.x) / 2, c1.y * 2 - (A.y + B.y) / 2);
                }
                for (double t = 0; t <= 1; t = t + tStep) {
                    results.push_back(bezierFormula(A, B, c1, t));
                }
            }
        }
        else {
            return generateCurve(start, end, controlVectors[0], includeC1, steps);
        }
    }
    return results;
};

void generateCurve(std::vector<Point2d>& points, Point2d start, Point2d end, std::vector<Point2d>& controlPoints, bool includeC1, int steps) {
    if (controlPoints.size() > 0) {
        if (controlPoints.size() > 1) {
            std::vector<Point2d> endpoints;
            endpoints.push_back(start);
            for (int i = 0; i < controlPoints.size() - 1; i++) {
                endpoints.push_back(midpoint(controlPoints[i], controlPoints[i + 1]));
            }
            endpoints.push_back(end);

            double tStep = 1.0 / steps;
            double headTarget = 0;
            for (int i = 0; i < endpoints.size() - 1; i++) {
                Point2d A = endpoints[i];
                Point2d B = endpoints[i + 1];
                Point2d c1 = controlPoints[i];
                if (includeC1) {
                    c1 = Point2d(c1.x * 2 - (A.x + B.x) / 2, c1.y * 2 - (A.y + B.y) / 2);
                }
                for (double t = 0; t <= 1; t = t + tStep) {
                    points.push_back(bezierFormula(A, B, c1, t));
                }
            }
        }
        else {
            generateCurve(points, start, end, controlPoints[0], includeC1, steps);
        }
    }
};

void generateCurve(std::vector<Point2d>& points, Point2d start, Vector2d end, std::vector<Vector2d>& controlVectors, bool includeC1, int steps) {
    if (controlVectors.size() > 0) {
        if (controlVectors.size() > 1) {
            std::vector<Point2d> endpoints;
            endpoints.push_back(start);
            for (int i = 0; i < controlVectors.size() - 1; i++) {
                endpoints.push_back(midpoint(controlVectors[i] + start, controlVectors[i + 1] + start));
            }
            endpoints.push_back(end + start);

            double tStep = 1.0 / steps;
            double headTarget = 0;
            for (int i = 0; i < endpoints.size() - 1; i++) {
                Point2d A = endpoints[i];
                Point2d B = endpoints[i + 1];
                Point2d c1 = controlVectors[i] + start;
                if (includeC1) {
                    c1 = Point2d(c1.x * 2 - (A.x + B.x) / 2, c1.y * 2 - (A.y + B.y) / 2);
                }
                for (double t = 0; t <= 1; t = t + tStep) {
                    points.push_back(bezierFormula(A, B, c1, t));
                }
            }
        }
        else {
            generateCurve(points, start, end, controlVectors[0], includeC1, steps);
        }
    }
};

std::vector<positionSet> curveHeadings(std::vector<Point2d>& points) {
    std::vector<positionSet> results;
    curveHeadings(results, points);
    return results;
};

void curveHeadings(std::vector<positionSet>& posSet, std::vector<Point2d>& points) {
    double head = 0;
    const int pSize = points.size();
    for (int i = 0; i < pSize; i++) {
        Point2d A = points[i];
        if (i < pSize - 1) {
            Point2d B = points[i + 1];
            head = normalizeAngle(atan2(B.y - A.y, B.x - A.x));
        }
        posSet.push_back({ A, head });
    }
};

positionSet predictLinear(positionSet start, Vector2d vel, double w, double t) {
    return { vel * t + start.p, start.head + w * t };
};

positionSet predictWithConstantTurning(positionSet start, Vector2d vel, double w, double t) {
    if (fabs(w) < MAXIMUM_DOUBLE_DIFFERENCE_FOR_EQUALS_SENSORDATA) {
        return predictLinear(start, vel, 0, t);
    }
    double v = vel.getMagnitude();
    double x = start.p.x + v * t * sin(w * t + start.head) / w + v * cos(w * t + start.head) / pow(w, 2);
    double y = start.p.y - v * t * cos(w * t + start.head) / w + v * sin(w * t + start.head) / pow(w, 2);
    return { Point2d(x, y), start.head + w * t };
};

//Path
//--------------------------------------------------------------------------------------------------
Path::Path() {
    //STUB
};

Path::Path(std::vector<positionSet>& pointsIn) {
    for (int i = 0; i < pointsIn.size(); i++) {
        addPointset(pointsIn[i]);
    }
};

std::vector<positionSet>& Path::getList() {
    return points;
};

void Path::setIndex(int i) {
    if (abs(i) >= size()) {
        i = sign(i) * (size() - 1);
    }
    if (i < 0) {
        i = size() + i;
    }
    internalIndex = i;
};

int Path::size() {
    return points.size();
};

positionSet Path::next() {
    if (hasNext()) {
        internalIndex++;
        return get(internalIndex);
    }
    return { Point2d(0, 0), 0 };
};

bool Path::hasNext() {
    if (internalIndex < size()) {
        return true;
    }
    return false;
};

int Path::index() {
    return internalIndex;
};

positionSet Path::get(int i) {
    if (abs(i) >= size()) {
        i = sign(i) * (size() - 1);
    }
    if (i < 0) {
        i = size() + i;
    }
    return points[i];
};

void Path::dropFront() {
    points.erase(points.begin(), points.begin() + internalIndex);
    points.shrink_to_fit();
    internalIndex = 0;
};

void Path::clear() {
    points.clear();
    internalIndex = 0;
}

void Path::addPointset(positionSet p) {
    points.push_back(p);
};

void Path::addPointset(Point2d p, double head, bool inDeg) {
    if (inDeg) { degToRad(head); }
    points.push_back({ p, head });
};


double Path::arclength(int start, int end) {
    int si = size();
    if (start < 0) {
        if (abs(start) < si) {
            start = si + start;
        }
        else {
            start = 0;
        }
    }
    if (end < 0) {
        if (abs(end) < si) {
            end = si + end;
        }
        else {
            end = 0;
        }
    }
    if (start >= si) { start = si - 1; }
    if (end >= si) { end = si - 1; }

    double result = 0;
    for (int i = start; i <= end - 1; i++) {
        result += Vector2d(points[i].p, points[i + 1].p).getMagnitude();
    }
    return result;
};

double Path::arclengthFromIndexTo(int end) {
    return arclength(internalIndex, end);
};

void Path::subpath(Path& pathIn, int start, int end) {
    for (int i = start; i < end; i++) {
        pathIn.addPointset(get(i));
    }
};

Path Path::subpath(int start, int end) {
    Path result;
    subpath(result, start, end);
    return result;
};

positionSet Path::operator [] (int i) {
    return get(i);
};

Path& Path::operator + (positionSet& p) {
    addPointset(p);
    return *this;
};

Path& Path::operator + (Path& p) {
    for (int i = 0; i < p.size(); i++) {
        addPointset(p[i]);
    }
    return *this;
};

std::ostream& operator << (std::ostream& os, Path& p) {
    os << "{";
    for (int i = 0; i < p.size(); i++) {
        os << p.get(i) << ", ";
    }
    os << "}";
    return os;
};