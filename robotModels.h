#ifndef ROBOT_MODELS_H
#define ROBOT_MODELS_H

/*
Name: robotModels.h
Written By: Carson Easterling

*/

#include "robotmath/robotmath.h"
#include "logger.h"
#include "navigation.h"
#include "v5.h"
#include "v5_vcs.h"

//Commands a velocity based on pos
//Timeouts are measured on the controller end but called from and terminated at the robot end
class MotionController {
protected:
    TargetPath* tPath = nullptr;

    Vector2d realtiveTargetVel = Vector2d(0, 0);
    double targetW = 0;

public:
    virtual void updateVel(double deltaT) = 0; //Update vel targets
    virtual positionSet predictNextPos(double deltaT) = 0; //Predict next location at +deltaT
    virtual bool isDone() = 0; //Is stopped at target?

    virtual void refresh(TargetPath* targetPath) {
        tPath = targetPath; 
    };

    Vector2d getVelocity() {
        return realtiveTargetVel; //Returns in Pct/Sec
    }

    double getAngularVelocity() {
        return targetW; //Radians
    }
};

//TODO Be able to specifiy in local (current) or grid coordnates
class CVController : public MotionController {
public:
    CVController(Vector2d realitiveTargetVelocity, double targetOmega) {
        realtiveTargetVel = realitiveTargetVelocity;
        targetW = targetOmega;
    }

    void updateVel(double deltaT) {}

    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), navigation.getVelocity(), navigation.getAngularVelocity(), deltaT);
    }

    bool isDone() {
        return false;
    }
};

class FeedForwardDriveController : public MotionController {
public:
    double linK;
    double linC;
    double angK;
    double angC;
    double linThres;
    double angThres;
    double angleUpdateThes;
    double motionAngularCone;
    double angularFrequency;

    //Recommended editables
    bool fwd = true;

    FeedForwardDriveController(double linearK, double linearC, double angularK, double angularC, 
                          double linearThreshold=1, double motionAngularLimit=degToRad(45), double noAngleUpdateThreshold=5.0, double turnAngularThreshold=degToRad(2)) {
        linK = linearK;
        linC = linearC;
        angK = angularK;
        angC = angularC;
        linThres = linearThreshold;
        angThres = turnAngularThreshold;
        angleUpdateThes = noAngleUpdateThreshold;
        motionAngularCone = motionAngularLimit;
        angularFrequency = (M_PI_2) / motionAngularLimit;
    }

    void setDirection(bool forward){
        fwd = forward;
    }

    void updateVel(double deltaT) {
        if (tPath == nullptr) {
            realtiveTargetVel = Vector2d(0, 0);
            targetW = 0;
            return;
        }

        positionSet location = navigation.getPosition();
        positionSet target = tPath->getTarget()->data;
        Vector2d errorV = Vector2d(location.p, target.p);

        double error = navigation.translateGlobalToLocal(errorV).getY(); ; //Returns the error in the forward direction of the robot
        double targetHead = Vector2d(1, 0).getAngle(errorV);

        //std::cout << "LIN ERROR" << error << ", " << radToDeg(targetHead) << std::endl;
        //std::cout << target.p << std::endl;

        double angularError = 0;
        if(errorV.getMagnitude() > angleUpdateThes){
            if(fwd){
                angularError = shortestArcToTarget(location.head, targetHead);
            } else {
                angularError = shortestArcToTarget(location.head, targetHead + M_PI);
            }
        }
        
        //Angular speed
        if(fabs(angularError) > angThres){
            targetW = angularError*angK + angC*sign(angularError);
        } else {
            targetW = 0;
        }

        double speed = 0;
        if(fabs(error) > linThres) { 
            speed = linK*error + linC*sign(error); 
            speed = speed * fmax(0, cos(angularFrequency*angularError));
        }
 
        realtiveTargetVel = Vector2d(0, speed);
    }

    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), navigation.getVelocity(), navigation.getAngularVelocity(), deltaT);
    }

    bool isDone() {
        if (tPath == nullptr) {
            return true;
        }

        positionSet location = navigation.getPosition();
        positionSet target = tPath->getTarget()->data;
        Vector2d errorV = Vector2d(location.p, target.p);

        //std::cout << errorV.getMagnitude() << std::endl;
        return (navigation.isLinearStopped() && navigation.isRotationalStopped() && (errorV.dot(navigation.getRobotNormalVector()) < linThres) && (errorV.getMagnitude() < angleUpdateThes));
    }
};

//KUp, kDown, linC, angK, angC, cruiseSpeed
class TrapizoidalDriveController : public FeedForwardDriveController {
protected:
    Point2d startPos = Point2d(0, 0);

public:
    double linKUp;
    double linKDown;
    double cruiseSpeed;

    //Prob could inherit from other class to reduce copyover
    TrapizoidalDriveController(double linearKUp, double linearKDown, double linearC, double angularK, double angularC, 
                          double cruiseSpeedIn, double linearThreshold=1, double motionAngularLimit=degToRad(45), double noAngleUpdateThreshold=5.0, double turnAngularThreshold=degToRad(2)) 
                          : FeedForwardDriveController(0, 0, angularK, angularC) {
        linKUp = linearKUp;
        linKDown = linearKDown;
        linC = linearC;
        angK = angularK;
        angC = angularC;
        linThres = linearThreshold;
        angThres = turnAngularThreshold;
        angleUpdateThes = noAngleUpdateThreshold;
        motionAngularCone = motionAngularLimit;
        angularFrequency = (M_PI_2) / motionAngularLimit;
        cruiseSpeed = cruiseSpeedIn;
    }

    void refresh(TargetPath* targetPath) override {
        MotionController::refresh(targetPath);
        startPos = navigation.getPosition().p; 
    };

    void updateVel(double deltaT) {
        if (tPath == nullptr) {
            realtiveTargetVel = Vector2d(0, 0);
            targetW = 0;
            return;
        }

        positionSet location = navigation.getPosition();
        positionSet target = tPath->getTarget()->data;
        Vector2d errorV = Vector2d(location.p, target.p);

        double error = navigation.translateGlobalToLocal(errorV).getY(); ; //Returns the error in the forward direction of the robot
        double targetHead = Vector2d(1, 0).getAngle(errorV);

        //std::cout << "LIN ERROR" << error << ", " << radToDeg(targetHead) << std::endl;
        //std::cout << target.p << std::endl;

        double angularError = 0;
        if(errorV.getMagnitude() > angleUpdateThes){
            if(fwd){
                angularError = shortestArcToTarget(location.head, targetHead);
            } else {
                angularError = shortestArcToTarget(location.head, targetHead + M_PI);
            }
        }
        
        //Angular speed
        if(fabs(angularError) > angThres){
            targetW = angularError*angK + angC*sign(angularError);
        } else {
            targetW = 0;
        }

        double speed = 0;
        if(fabs(error) > linThres) { 
            Vector2d reverseError = Vector2d(startPos, location.p);
            speed = fmin(linKDown*fabs(error) + linC, cruiseSpeed);
            speed = fmin(speed, linKUp*reverseError.getMagnitude() + linC);
            speed = speed * fmax(0, cos(angularFrequency*angularError));
            speed = speed * sign(error);
            //std::cout << speed << std::endl;
        }
        
        realtiveTargetVel = Vector2d(0, speed);
    }
};


class FeedForwardTurnController : public MotionController {
private:
    int direction = 0; //0 = Shortest, 1 = Forced CCW, 2 = Forced CW
public:
    double p;
    double i;
    double c;
    double thres;
    double accumulator = 0;

    FeedForwardTurnController(double p, double i, double constant, double threshold=degToRad(1)){
        this->p = p;
        this->i = i;
        c = constant;
        thres = threshold;
    }

    void setDirection(int d){
        direction = clamp(d, 0, 2);
    }

    double determineError() {
        if (tPath == nullptr) {
            return 0;
        }

        positionSet location = navigation.getPosition();
        positionSet target = tPath->getTarget()->data;

        double error = shortestArcToTarget(location.head, target.head);

        switch (direction) {
        case 1:
            if (error < 0) {
                error = M_2PI + error;
            }
            break;
        case 2:
            if (error > 0) {
                error =  error - M_2PI;
            }
            break;
        }

        return error;
    }

    void updateVel(double deltaT) {
        if (tPath == nullptr) {
            realtiveTargetVel = Vector2d(0, 0);
            targetW = 0;
            return;
        }

        double error = determineError();

        // PID Integral math
        // Only activates when |error| < ~10 degrees
        if (fabs(error) < 0.15) {
            accumulator += error;
            // Reset accumulator if value gets too large to avoid integral windup
            if (fabs(accumulator) > M_PI) {
                accumulator = 0;
            }
        } else {
            accumulator = 0;
        }

        realtiveTargetVel = Vector2d(0, 0);

        if(fabs(error) > thres){
            targetW = error*p + i*accumulator + c*sign(error);
        } else {
            accumulator = 0;
            targetW = 0;
        }
    }
     
    bool isDone() {
        if (tPath == nullptr) {
            return true;
        }

        double error = determineError();
        return (navigation.isRotationalStopped() && (fabs(error) < thres));
    }
    
    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), Vector2d(0, 0), navigation.getAngularVelocity(), deltaT);
    }
};

class StraightPathController : public MotionController {
private:
    FeedForwardDriveController* linCont;
    FeedForwardTurnController* rotCont;
    bool turning = true;
public:
    StraightPathController(FeedForwardDriveController* linearController, FeedForwardTurnController* turnController){
        linCont = linearController;
        rotCont = turnController;
    }

    void updateVel(double deltaT){
        if (tPath == nullptr) {
            realtiveTargetVel = Vector2d(0, 0);
            targetW = 0;
            return;
        }

        if(!isDone()){
            NodePS* t = tPath->getTarget();
            if(turning){
                double newHead = Vector2d(1, 0).getAngle(Vector2d(navigation.getPosition().p, t->data.p));
                t->data.head = newHead;

                rotCont->updateVel(deltaT);
                realtiveTargetVel = rotCont->getVelocity();
                targetW = rotCont->getAngularVelocity();

                if(rotCont->isDone()){
                    turning = false;
                    linCont->refresh(tPath);
                }
            } else {
                linCont->updateVel(deltaT);
                realtiveTargetVel = linCont->getVelocity();
                targetW = linCont->getAngularVelocity();

                if(linCont->isDone()){
                    turning = true;
                    rotCont->refresh(tPath);
                    tPath->shiftTarget();
                }
            }
        } else {
            realtiveTargetVel = Vector2d(0, 0);
            targetW = 0;
        }
    }
    
    positionSet predictNextPos(double deltaT){
        if(turning){
            return rotCont->predictNextPos(deltaT);
        } else {
            return linCont->predictNextPos(deltaT);
        }
    }

    bool isDone(){
        if (tPath == nullptr) {
            return true;
        }

        return tPath->pointingToLastTarget() && !turning && linCont->isDone();
    }

    void refresh(TargetPath* targetPath) {
        turning = true;
        MotionController::refresh(targetPath);
    };
};

//TODO support heading independece option for XDrive type
class CurvePathController : public MotionController {
private:
    double speed;
    double kTurning;
    double lookAheadDistance;

public:
    CurvePathController(double speed, double kTurning, double lookAheadDistance) {
        this->speed = speed;
        this->kTurning = kTurning;
        this->lookAheadDistance = lookAheadDistance;
    }

    void updateVel(double deltaT){
        positionSet currentPos = navigation.getPosition();
        NodePS* t = tPath->getTarget();
        positionSet target = t->data;
        Vector2d error = Vector2d(currentPos.p, target.p);

        while (error.getMagnitude() < lookAheadDistance) {
            tPath->shiftTarget();
            t = tPath->getTarget();
            target = t->data;
            error = Vector2d(currentPos.p, target.p);
        }

        if (t->hasNext()) {
            positionSet nextTarget = t->getNext()->data;
            Vector2d nextError = Vector2d(target.p, nextTarget.p);

            double a = error.getMagnitude();
            double b = nextError.getMagnitude();
            double c = Vector2d(currentPos.p, nextTarget.p).getMagnitude();

            double q = (a * a + b * b - c * c) / (2 * a * b);
            if (q != 1) {
                double R = c / (2 * sqrt(1 - q * q));
                double shiftTheta = 0.5 * acos((a * a - 2 * R * R) / (-2 * R * R));
                shiftTheta = abs(shiftTheta) * -1 * sign(error.getAngle(nextError));

                t->data.head = Vector2d(1, 0).getAngle(error) + shiftTheta;
            }
            else {
                t->data.head = Vector2d(1, 0).getAngle(error);
            }

            double hError = shortestArcToTarget(currentPos.head, t->data.head);
            realtiveTargetVel = Vector2d(0, speed).scale(cos(hError));
            targetW = kTurning * hError;
        }
        else {
            double hError = shortestArcToTarget(currentPos.head, t->data.head);
            realtiveTargetVel = Vector2d(0, speed).scale(cos(hError));
            targetW = kTurning * hError;
        }
    }
    
    positionSet predictNextPos(double deltaT){
        return { Point2d(0, 0), 0 };
    }

    bool isDone(){
        return false;
    }
};

// From "Controls Engineering in FRC" Chapter 8.8
// The values b=2 and zeta=0.7 when using meters and radians
// An extra constructor is provided that uses these constants for *inches* and radians
//
// Units:
// - zeta = 1/angle
// - b = angle^2/length^2
// 
// Constraints:
// - b > 0
// - 0 < zeta < 1
class RamseteController : public MotionController {
private:
    double zeta, b, fwdSpeed, rotSpeed;
    double samplePos = 0;
    bool holonomic;

    static double sincf(double x) {
        // lim x->0 sin(x)/x = 1 (by L'Hôpital's rule)
        if (fabs(x) < 1e-6) {
            return 1;
        }
        return sinf(x) / x;
    }

    positionSet samplePath() {
        NodePS* current = tPath->getTarget();
        if (!current) {
            return { Point2d(0, 0), 0 };
        }
        
        NodePS* next = current->getNext();
        // At end
        if (!next) {
            return current->data;
        }

        Vector2d dir = next->data.p - current->data.p;
        double length = dir.getMagnitude();

        // If we are past the next point, shift and retry
        if (samplePos > length) {
            samplePos -= length;
            tPath->shiftTarget();
            return samplePath();
        }

        // Lerp position
        double t = samplePos / length;
        Vector2d pos = dir * t;

        double heading = 0;

        // 3 DOF drive trains can point in any direction
        if (holonomic) { // X drive
            // Lerp heading
            double currHead = normalizeAngle(current->data.head);
            double nextHead = normalizeAngle(next->data.head);

            // Go the other way because its faster
            if (nextHead - currHead > M_PI) {
                nextHead = -nextHead;
            }

            heading = (currHead * (1 - t)) + (nextHead * t);
        } else { // Tank drive
            // Otherwise we must be pointing tangent to the path
            heading = normalizeAngle(dir.getAngle(Vector2d(1, 0)));
        }

        return { Point2d(pos.getX(), pos.getY()), heading };
    }
public:
    RamseteController(double fwdSpeed, double rotSpeed, bool holonomic) : RamseteController(0.7, 2.0 / (100 * 100 * 2.54 * 2.54), fwdSpeed, rotSpeed, holonomic) {}

    RamseteController(double zeta, double b, double fwdSpeed, double rotSpeed, bool holonomic) {
        this->zeta = zeta;
        this->b = b;
        this->fwdSpeed = fwdSpeed;
        this->rotSpeed = rotSpeed;
        this->holonomic = holonomic;
    }

    void updateVel(double deltaT) override {
        positionSet target = samplePath();
        positionSet current = navigation.getPosition();

        // Get offset from path
        double dx = target.p.x - current.p.x;
        double dy = target.p.y - current.p.y;
        double dTheta = normalizeAngle(target.head) - normalizeAngle(current.head);

        // Transform to robot frame
        double ex = (dx * cosf(target.head)) + (dy * sinf(target.head));
        double ey = -(dx * sinf(target.head)) + (dy * cosf(target.head));
        double eTheta = dTheta;

        // Desired linear/angular velocities
        // You could add motion profiles or other fancy stuff here
        // Going to alias for now since we do not have velocity information in the path
        double vd = this->fwdSpeed;
        double wd = this->rotSpeed;

        // Time-varying gain, do not change (change b or zeta instead)
        double k = 2 * this->zeta * sqrtf((wd * wd) + (b * (vd * vd)));

        this->realtiveTargetVel = Vector2d(0, (vd * cosf(eTheta)) + (k * ex));
        this->targetW = wd + (k * eTheta) + (b * vd * sincf(eTheta) * ey);

        // Update sampling position but only progress if we are close
        if (Vector2d(current.p, target.p).getMagnitude() < 3) {
            samplePos += deltaT * vd;
        }
    }

    positionSet predictNextPos(double deltaT) override {
        auto pos = navigation.getPosition();
        return predictLinear(pos, realtiveTargetVel.getRotatedVector(M_PI_2 - normalizeAngle(pos.head)), targetW, deltaT);
    }

    bool isDone() override {
        return tPath->pointingToLastTarget();
    }
};

class PurePursitController : public MotionController {
private:
    double speed, lookahead, pTurn;
public:
    PurePursitController(double speed, double lookahead, double pTurn) {
        this->speed = speed;
        this->lookahead = lookahead;
        this->pTurn = pTurn;
    }

    void updateVel(double deltaT) override {
        this->realtiveTargetVel = Vector2d(0, 0);
        this->targetW = 0;

        if (!tPath) {
            return;
        }

        NodePS* prev = tPath->getTarget();
        if (!prev) { 
            return;
        }
        NodePS* curr = prev->getNext();
        if (!curr) {
            return;
        }

        Vector2d delta = curr->data.p - prev->data.p;
        positionSet position = navigation.getPosition();

        Point2d target = curr->data.p;
        // Passed prev point, apply lookahead
        if ((position.p - prev->data.p).dot(delta) > 0) {
            double l = lookahead + (position.p - prev->data.p).project(delta).getMagnitude();
            auto offset = delta * (l / delta.getMagnitude());
            target = prev->data.p;
            target = Point2d(target.x + offset.getX(), target.y + offset.getY());
        }

        double angle = navigation.getRobotNormalVector().getAngle(target - position.p);

        // Passed curr point, go to next point
        if ((target - curr->data.p).dot(delta) > 0) {
            tPath->shiftTarget();
        }

        this->realtiveTargetVel = Vector2d(0, speed);
        this->targetW = pTurn * angle;

        if (!curr->hasNext()) {
            target = curr->data.p;
            // double k = (position.p - target).getMagnitude() / delta.getMagnitude();
            realtiveTargetVel = realtiveTargetVel/* * fmax(0.5, fmin(1, k * k))*/;
        }
    }
    
    positionSet predictNextPos(double deltaT) override {
        auto pos = navigation.getPosition();
        return predictLinear(pos, realtiveTargetVel.getRotatedVector(M_PI_2 - normalizeAngle(pos.head)), targetW, deltaT);
    }
    
    bool isDone() override {
        return tPath->pointingToLastTarget();
    }
};

//https://wiki.purduesigbots.com/software/control-algorithms/ramsete
//https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit

/*
    Motion Controller Ideas:
        Target CV
        Linear PID
        Linear PC
        Rotation PID
        Double PID, one to control velocity (Using voltage), one to set target velocity -- https://bryceautomation.com/index.php/2021/12/11/cascading-pid/
        Curve - Limits speed based on curvature - Could use horizontal acceleration to calculate curve radius
        Independent Heading w/Motion (For X Drive)
        Turn Only
        Gain Schedurling
        Trapzoidal Motion Profile
        Pure Pursuit - https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
        Path with defined velocity points

        Could make class of each then have seperate implementations for linear and rotation that get the results and set to motors differently
        Could also have classes that combine the two
*/



//TODO Convert Units/Second to Pct; Employ PID with navigation to ensure consistant velocity control; Robot type nees to take in wheelRadius data to calculate; Take in robot wheel radius to ensure proper rotation
//Aceleration control also happens here to prevent slipage

void setM(vex::motor m, double speed, vex::velocityUnits uni = vex::velocityUnits::pct) {
    m.setVelocity(speed, uni);
    if (fabs(speed) < 0.1) {
        m.stop();
    }
    else {
        m.spin(vex::directionType::fwd);
    }
}

void setM(vex::motor_group m, double speed, vex::velocityUnits uni = vex::velocityUnits::pct) {
    m.setVelocity(speed, uni);
    if (fabs(speed) < 0.1) {
        m.stop();
    }
    else {
        m.spin(vex::directionType::fwd);
    }
}

//TODO Have robot class select controllers to move through target path

//Converts a vel to motor commands
CVController defaultController = CVController(Vector2d(0, 0), 0);
class RobotType {
private:
    const int BaseMotorRPM = 3600;
    MotionController* controller;

protected:
    double length; //Distance back to front
    double width; //Distance from left most point to right most point
    double wheelRadius; //Radius of drive wheel
    double driveBaseRadius; //Distance from center of rotation to middle of drive wheels 
    double gearRatio; //Product of motor gear and drivetrain ratios

    //Max speed, and acceleration commands
    double maxLinearLimit = 100; //Pct
    double maxAngularLimit = 100; //Pct
    double maxMotorAcceleration = 200; //Pct /s 

    MotionController* const getController() {
        return controller;
    }

public:
    bool setMotorControls = true;
    bool controllerActive = true;

    //Length is front to back, width is side to side, wheelRadius is the radius of a drive wheel, driveBaseWidth is the distance from midwheel to the other side midwheel, gear ratio is in_out of both motor and drivetrain multiplied, maxSpeedInchesPerSecondIn at 100 percent as measured: -1 relies on math from motor
    RobotType(double lengthIn, double widthIn, double wheelRadiusIn, double driveBaseWidthIn, double gearRatio_in_out) {
        motion(nullptr, &defaultController);
        length = lengthIn;
        width = widthIn;
        gearRatio = gearRatio_in_out;
        wheelRadius = wheelRadiusIn;
        driveBaseRadius = driveBaseWidthIn * 0.5;
    }

    void motion(TargetPath* target, MotionController* newController) {
        newController->refresh(target);
        controller = newController;
    }

    positionSet predictNextPos(double deltaT) {
        return getController()->predictNextPos(deltaT);
    }

    int isDone() {
        return getController()->isDone();
    }

    void setMaxLinearSpeed(double x){
        maxLinearLimit = fclamp(x, 0, 100);
    }

    void setMaxAngularSpeed(double x){
        maxAngularLimit = fclamp(x, 0, 100);
    }

    double getMaxLinearSpeed(){
        return maxLinearLimit;
    }

    double getMaxAngularSpeed(){
        return maxAngularLimit;
    }

    double getMaxMotorAcceleration(){
        return maxMotorAcceleration;
    }
    
    //Ensure Controller.updateVel is called at start of function
    virtual void updateMotors(double deltaT) = 0; //=0 requires an overrider in derived classes
};


class TankDriveType : public RobotType {
private:
    vex::motor_group* leftSide;
    vex::motor_group* rightSide;

public:
    TankDriveType(vex::motor_group* leftSideArg, vex::motor_group* rightSideArg, double lengthIn, double widthIn, double wheelRadiusIn, double driveBaseWidthIn, double gearRatio_in_out) :
        RobotType(lengthIn, widthIn, wheelRadiusIn, driveBaseWidthIn, gearRatio_in_out) {
        leftSide = leftSideArg;
        rightSide = rightSideArg;
    }

    void updateMotors(double deltaT) {
        double speed = 0;
        double w = 0;

        if(controllerActive){
            getController()->updateVel(deltaT);
            speed = getController()->getVelocity().getY();
            w = getController()->getAngularVelocity();
        }
        
        speed = fclamp(speed, -maxLinearLimit, maxLinearLimit);
        w = fclamp(w, -maxAngularLimit, maxAngularLimit);

        double left = speed - w;
        double right = speed + w;

        double d = fabs(right) - fabs(left);
        if(d > 0.5){
            if (fabs(right) > 100) {
                left = left - (fabs(right) - 100) * sign(left);
                right = 100 * sign(right);
            }
        } else if (d < 0.5){
            if (fabs(left) > 100) {
                right = right - (fabs(left) - 100) * sign(left);
                left = 100 * sign(left);
            }
        }

        //TODO accel controls

        if (setMotorControls) {
            setM(*leftSide, left);
            setM(*rightSide, right);
        }
    }
};
/*
class XDriveType : public RobotType {
protected:
    vex::motor_group* NWMotor;
    vex::motor_group* NEMotor;
    vex::motor_group* SWMotor;
    vex::motor_group* SEMotor;

public:
    XDriveType(vex::motor_group* NWMotorArg, vex::motor_group* NEMotorArg, vex::motor_group* SWMotorArg, vex::motor_group* SEMotorArg) : RobotType() {
        NEMotor = NEMotorArg;
        SEMotor = SEMotorArg;
        NWMotor = NWMotorArg;
        SWMotor = SWMotorArg;
        maxLinearLimit = 141;
    }

    void updateMotors() {
        getController()->updateVel();

        double speed = getController()->getVelocity().getMagnitude();
        double travelAngle = Vector2d(1, 0).getAngle(getController()->getVelocity());
        double omega = 0.25 * getController()->getAngularVelocity();

        double NESW_Coefficent = (sin(travelAngle) - cos(travelAngle)) / (2 * sqrt(2));
        double NWSE_Coefficent = (sin(travelAngle) + cos(travelAngle)) / (2 * sqrt(2));

        //-, +, -, + to go CCW
        //NW, NE, SW, SE
        double speeds[4] = { NWSE_Coefficent * speed - omega,  //NW
                            NESW_Coefficent * speed + omega,  //NE
                            NESW_Coefficent * speed - omega,  //SW
                            NWSE_Coefficent * speed + omega }; //SE

        int iMax = -1;
        double maxDiff = 0;
        for (int i = 0; i < 4; i++) {
            double diff = abs(speeds[i]) - getMaxMotorVel();
            if (diff > 0) {
                if (diff > maxDiff) {
                    iMax = i;
                    maxDiff = diff;
                }
            }
        }

        if (iMax > -1) {
            double sol = sign(speeds[iMax]) * getMaxMotorVel();
            if (iMax == 0) {
                speed = (sol + omega) / NWSE_Coefficent;
            }
            else if (iMax == 1) {
                speed = (sol - omega) / NESW_Coefficent;
            }
            else if (iMax == 2) {
                speed = (sol + omega) / NESW_Coefficent;
            }
            else {
                speed = (sol - omega) / NWSE_Coefficent;
            }
        }



        if (setMotorControls) {
            setM(*NEMotor, NESW_Coefficent * speed + omega);
            setM(*NWMotor, NWSE_Coefficent * speed - omega);
            setM(*SEMotor, NWSE_Coefficent * speed + omega);
            setM(*SWMotor, NESW_Coefficent * speed - omega);
        }
    }
};
*/

#endif