#include "robotmath/robotmath.h"

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
                endpoints.push_back(Point2d::midpoint(controlPoints[i], controlPoints[i + 1]));
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
                endpoints.push_back(Point2d::midpoint(controlVectors[i] + start, controlVectors[i + 1] + start));
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
                endpoints.push_back(Point2d::midpoint(controlPoints[i], controlPoints[i + 1]));
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
                endpoints.push_back(Point2d::midpoint(controlVectors[i] + start, controlVectors[i + 1] + start));
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
    double x = start.p.x + (v * t * sin(w * t + start.head) / w);// +(v * cos(w * t + start.head) / pow(w, 2));
    double y = start.p.y - (v * t * cos(w * t + start.head) / w);// +(v * sin(w * t + start.head) / pow(w, 2));
    return { Point2d(x, y), start.head + w * t };
};

double calculateRadiusOfCurvature(Point2d start, Point2d middle, Point2d end) {
    double a = Vector2d(start, middle).getMagnitude();
    double b = Vector2d(middle, end).getMagnitude();
    double c = Vector2d(start, end).getMagnitude();

    double q = (a * a + b * b - c * c) / (2 * a * b);
    if (q != 1) {
        double R = c / (2 * sqrt(1 - q * q));
        return R * sign(Vector2d(start, middle).getAngle(Vector2d(middle, end))); //+ if Circle is going CCW
    }
    else {
        return 0;
    }
};
