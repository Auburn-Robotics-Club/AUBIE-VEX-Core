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
#include "../v5.h"
#include "../v5_vcs.h"

//Gain scheduling
//Different control mode type each get a class implementation
//Prediction of next location based on model class

#endif