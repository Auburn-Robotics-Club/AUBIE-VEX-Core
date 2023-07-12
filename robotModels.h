#ifndef ROBOT_MODELS_H
#define ROBOT_MODELS_H

/*
Name: robotModels.h
Written By: Carson Easterling

*/

#include "robotmath.h"
#include "logger.h"
#include "navigation.h"
#include "v5.h"
#include "v5_vcs.h"

//Commands a velocity based on pos
class MotionController {
protected:
    Vector2d realtiveTargetVel = Vector2d(0, 0);
    double targetW = 0;

public:
    virtual void updateVel() = 0;
    virtual positionSet predictNextPos(double deltaT) = 0;
    virtual int isDone() = 0; //0 - Not Done, 1 - Done, 2 - Timeout

    Vector2d getVelocity() {
        return realtiveTargetVel; //Returns in Units/Sec
    }

    double getAngularVelocity() {
        return targetW; //Radians
    }

    //TODO Timeout structure, accel controls

};

class CVController : public MotionController {
public:
    CVController(Vector2d realitiveTargetVelocity, double targetOmega) {
        realtiveTargetVel = realitiveTargetVelocity;
        targetW = targetOmega;
    }

    void updateVel() {}

    positionSet predictNextPos(double deltaT) {
        return predictLinear(navigation.getPosition(), navigation.getVelocity(), navigation.getAngularVelocity(), deltaT);
    }

    int isDone() {
        return 0;
    }
};

//https://wiki.purduesigbots.com/software/control-algorithms/ramsete
//https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit

/*
    Motion Controller Ideas:
        CV
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

//Converts a vel to motor commands
class RobotType {
private:
    const int baseMotorRPM = 3600; //Does not involve the 36:1, 18:1, 6:1 gear ratios
    MotionController* controller;

protected:
    double wheelRadius;
    double driveBaseRadius;

    //TODO Acceleration commands

    MotionController* const getController() {
        return controller;
    }

public:
    bool setMotorControls = true;

    RobotType(double wheelRadiusIn, double driveBaseRadiusIn, double gearRatio_in_out) {
        setController(new CVController(Vector2d(0, 0), 0));
        wheelRadius = wheelRadiusIn;
        driveBaseRadius = driveBaseRadiusIn;
    }

    void setController(MotionController* newController) {
        delete controller;
        controller = newController;
    }

    positionSet predictNextPos(double deltaT) {
        return getController()->predictNextPos(deltaT);
    }

    int isDone() {
        return getController()->isDone();
    }

    //Ensure Controller.updateVel is called at start of function
    virtual void updateMotors() = 0; //=0 requires an overrider in derived classes
};

/**
//TODO implement base constructor
class TankDriveType : public RobotType {
private:
    vex::motor_group* leftSide;
    vex::motor_group* rightSide;

public:
    TankDriveType(vex::motor_group* leftSideArg, vex::motor_group* rightSideArg) {
        leftSide = leftSideArg;
        rightSide = rightSideArg;
    }

    void updateMotors() {
        getController()->updateVel();

        double speed = getController()->getVelocity().getY();
        double w = getController()->getAngularVelocity();
        double maxMotorVel = getMaxMotorVel();

        double left = speed - w;
        double right = speed + w;

        if (fabs(right) > maxMotorVel) {
            left = left - (fabs(right) - maxMotorVel) * sign(left);
            right = maxMotorVel * sign(right);
        }
        if (fabs(left) > maxMotorVel) {
            right = right - (fabs(left) - maxMotorVel) * sign(left);
            left = maxMotorVel * sign(left);
        }

        if (setMotorControls) {
            setSide(*leftSide, left);
            setSide(*rightSide, right);
        }
    }
};

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