#include <math.h>

/*
class FancyPIDController {
private:
    bool wrapping;
    double wrapMin, wrapDistance;
    double target;
    double accumulator;
    static const int BUFFER_SIZE = 0xFF; // MUST be one less than an exponent of 2
    double pastError[BUFFER_SIZE + 1];
    int pastErrorIndex;
    double time;
public:
    double P, I, D;
    double IZone, IMax;
    double Threshold;
    bool UseCircularBufferIntegration = true;
    FancyPIDController();
    FancyPIDController(double wrapMin, double wrapMax);
    void setTarget(double target);
    double update(double measurement, double deltaTime);
    bool isAtTarget();
};

FancyPIDController::FancyPIDController() {
    wrapping = false;
}

FancyPIDController::FancyPIDController(double min, double max) {
    wrapping = true;
    wrapMin = min;
    wrapDistance = max - min;
}

void FancyPIDController::setTarget(double target) {
    if (wrapping) {
        target = euclidMod(target - wrapMin, wrapDistance) + wrapMin;
    }
    this->target = target;
}

double FancyPIDController::update(double measurement, double deltaTime) {
    time += deltaTime;
    double error = target - measurement;
    double lastError = pastError[(pastErrorIndex - 1) & BUFFER_SIZE];

    if (wrapping) {
        measurement = euclidMod(measurement - wrapMin, wrapDistance) + wrapMin;
        error = target - measurement;
        if (fabs(error) >= wrapDistance / 2) {
            error = wrapDistance - error;
            error = euclidMod(error, wrapDistance);
        }
    }
    
    // P
    double out = P * error;

    // I
    if (fabs(error) < IZone) {
        out += I * accumulator;
        accumulator += error;
        if (UseCircularBufferIntegration){
            // This will subtract the oldest error value 
            accumulator -= pastError[(pastErrorIndex + 1) & BUFFER_SIZE];
        }
        // Reset if accumulator is too large
        if (fabs(accumulator) > IMax) {
            accumulator = 0;
        }
    } else {
        accumulator = 0;
    }
    pastError[pastErrorIndex] = error;
    pastErrorIndex++;
    pastErrorIndex &= 0xFF;

    // D
    out += D * (error - lastError) / deltaTime;
    return out;
}

bool FancyPIDController::isAtTarget() {
    return fabs(pastError[pastErrorIndex]) < Threshold;
}

// Modulus that behaves for negative x
static double euclidMod(double x, double y) {
    return x - (y * floor(x / y));
}*/