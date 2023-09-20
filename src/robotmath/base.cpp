#include "base.h"

//Misc Functions
//--------------------------------------------------------------------------------------------------

//Returns minimum number
int min(int a, int b) {
    if (a > b) { return b; }
    return a;
}

double fmin(double a, double b) {
    if (a > b) { return b; }
    return a;
}

//Returns maximum number
int max(int a, int b) {
    if (a < b) { return b; }
    return a;
}

double fmax(double a, double b) {
    if (a < b) { return b; }
    return a;
}

//Clamps output between min and max
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