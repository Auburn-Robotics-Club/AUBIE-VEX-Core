#ifndef ROBOT_MODELS_H
#define ROBOT_MODELS_H

/*
Name: robotModels.h
Written By: Carson Easterling

TODO

!!!!!Keep Linear PC but make rotation PID

Smooth path transitions between points when setting veloctity
build in support releying current vel for accel if jumping becoems an issue
*/

#include "robotmath.h"
#include "logger.h"
#include "navigation.h"
#include "v5.h"
#include "v5_vcs.h"

//Different control mode type each get a class implementation
//Prediction of next location based on model class
//Timeouts should be handled my motion controller
//Also each should predict next position using their drive modes to aide with filtering

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

        Could make class of each then have seperate implementations for linear and rotation that get the results and set to motors differently
        Could also have classes that combine the two
*/
#endif