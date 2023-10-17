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
    Vector2d realtiveTargetVel = Vector2d(0, 0);
    double targetW = 0;

public:
    virtual void updateVel(double deltaT) = 0; //Update vel targets
    virtual positionSet predictNextPos(double deltaT) = 0; //Predict next location at +deltaT
    virtual bool isDone() = 0; //Is stopped at target?
    virtual void refresh() = 0;

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
    void refresh() {}

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

    //Recommended editables
    bool fwd = true;

    FeedForwardDriveController(double linearK, double linearC, double angularK, double angularC, 
                          double linearThreshold=1, double motionAngularLimit=degToRad(20), double noAngleUpdateThreshold=5, double turnAngularThreshold=degToRad(2)) {
        linK = linearK;
        linC = linearC;
        angK = angularK;
        angC = angularC;
        linThres = linearThreshold;
        angThres = turnAngularThreshold;
        angleUpdateThes = noAngleUpdateThreshold;
        motionAngularCone = motionAngularLimit;
    }

    void setDirection(bool forward){
        fwd = forward;
    }

    void updateVel(double deltaT) {
        positionSet location = navigation.getPosition();
        positionSet target = navigation.getTarget();
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

        double speed = 0;
        if(fabs(error) > linThres) { 
            if (fabs(angularError) < motionAngularCone){
                speed = linK*error + linC*sign(error); 
            }
        }
        realtiveTargetVel = Vector2d(0, speed);

        //Angular speed
        if(fabs(angularError) > angThres){
            targetW = angularError*angK + angC*sign(angularError);
        } else {
            targetW = 0;
        }
    }

    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), navigation.getVelocity(), navigation.getAngularVelocity(), deltaT);
    }

    bool isDone() {
        //IF no next targets or if next target when shifting reset timeouts (prob handle in diff function)
        positionSet location = navigation.getPosition();
        positionSet target = navigation.getTarget(); //TODO What happens if target list is empty? - 
        Vector2d errorV = Vector2d(location.p, target.p);

        return (navigation.isLinearStopped() && navigation.isRotationalStopped() && (errorV.getMagnitude() < linThres));
    }

    void refresh(){};
};

class FeedForwardTurnController : public MotionController {
private:
    int direction = 0; //0 = Shortest, 1 = Forced CCW, 2 = Forced CW
public:
    double p;
    double c;
    double thres;

    FeedForwardTurnController(double k, double constant, double threshold=degToRad(1)){
        p = k;
        c = constant;
        thres = threshold;
    }

    void setDirection(int d){
        direction = clamp(d, 0, 2);
    }

    double determineError() {
        positionSet location = navigation.getPosition();
        positionSet target = navigation.getTarget();

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
        double error = determineError();

        realtiveTargetVel = Vector2d(0, 0);

        if(fabs(error) > thres){
            targetW = error*p + c*sign(error);
        } else {
            targetW = 0;
        }
    }
     
    bool isDone() {
        double error = determineError();
        return (navigation.isRotationalStopped() && (abs(error) < thres));
    }
    
    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), Vector2d(0, 0), navigation.getAngularVelocity(), deltaT);
    }

    void refresh(){};
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
        if(!isDone()){
            if(turning){
                rotCont->updateVel(deltaT);
                realtiveTargetVel = rotCont->getVelocity();
                targetW = rotCont->getAngularVelocity();

                if(rotCont->isDone()){
                    turning = false;
                    linCont->refresh();
                }
            } else {
                linCont->updateVel(deltaT);
                realtiveTargetVel = linCont->getVelocity();
                targetW = linCont->getAngularVelocity();

                if(linCont->isDone()){
                    turning = true;
                    rotCont->refresh();
                    navigation.shiftTarget();
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
        return navigation.pointingToLastTarget() && !turning && linCont->isDone();
    }

    void refresh(){
        rotCont->refresh();
        linCont->refresh();
    }
};

//TODO support heading independece option for XDrive type
class CurvePathController : public MotionController {
private:
    double linK;
    double linC;
    double rotK;
    double rotC;

public:
    CurvePathController(double linearK, double linearC, double rotationalK, double rotationalC){
        linK = linearK;
        linC = linearC;
        rotK = rotationalK;
        rotC = rotationalC;
    }

    void updateVel(double deltaT){
        //Point2d currentPos = navigation.getPosition();
        //Point2d target = navigation.getTarget();
        //Point2d nextTarget = navigation.getNextTarget();


    }
    
    positionSet predictNextPos(double deltaT){

    }

    bool isDone(){
        
    }

    void refresh(){
        
    }
};
/*      if(targetPathIndex < targetPath.size() - 1){
        double a = error.getMagnitude();
        double b = getNextGlobalError().getMagnitude();
        double c = Vector(currentPos.p, nextTarget.p).getMagnitude();

        double q = (a*a + b*b - c*c)/(2*a*b);
        if(q != 1){
          double R = c / (2*sqrt(1-q*q));
          double shiftTheta = 0.5*acos((a*a-2*R*R)/(-2*R*R));
          shiftTheta = abs(shiftTheta) * -1 * sign(error.getAngle(getNextGlobalError()));

          setCurrentTargetTargetHeading(Vector(1, 0).getAngle(error) + shiftTheta);
        }else{
          setCurrentTargetTargetHeading(Vector(1, 0).getAngle(error));
        }
      }else{
        if(error.getMagnitude() > noNewTurnRadius){
          double targetHead = Vector(1, 0).getAngle(error);
          if(!forward){
            targetHead = targetHead + PI;
            targetHead = normalizeAngle(targetHead);
          }
          setCurrentTargetTargetHeading(targetHead);
        }
      }
      
      //Curve PID
        angleAdjustSpeed = max(0, 1 - 32.828*abs(hError)*abs(hError)); // TODO Test (+-10  degrees results in  as parabolic)
        //angleAdjustSpeed = 1 - (2*abs(hError)/PI)*(errorRadius/error.getMagnitude()); // TODO TEST
        targetVelocity.scale(angleAdjustSpeed);
      
      */

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
    if (speed == 0) {
        m.stop();
    }
    else {
        m.spin(vex::directionType::fwd);
    }
}

void setSide(vex::motor_group m, double speed, vex::velocityUnits uni = vex::velocityUnits::pct) {
    m.setVelocity(speed, uni);
    if (speed == 0) {
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
        setController(&defaultController);
        length = lengthIn;
        width = widthIn;
        gearRatio = gearRatio_in_out;
        wheelRadius = wheelRadiusIn;
        driveBaseRadius = driveBaseWidthIn * 0.5;
    }

    void setController(MotionController* newController) {
        newController->refresh();
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
            setSide(*leftSide, left);
            setSide(*rightSide, right);
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
            setSide(*NEMotor, NESW_Coefficent * speed + omega);
            setSide(*NWMotor, NWSE_Coefficent * speed - omega);
            setSide(*SEMotor, NWSE_Coefficent * speed + omega);
            setSide(*SWMotor, NESW_Coefficent * speed - omega);
        }
    }
};
*/

#endif