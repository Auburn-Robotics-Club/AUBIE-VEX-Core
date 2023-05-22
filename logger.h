#pragma once

/*
Name: logger.h
Written By: Carson Easterling
*/

#include <iostream>
#include <sstream>
#include "robotmath.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Logging Constants
//--------------------------------------------------------------------------------------------------
const int TIMEOUT_DATA = 0;
const int GRAPH_DATA = 1;
const int META_DATA = 2;
const int POINT_DATA = 3;
const int VECTOR_DATA = 4;
const int POINTSET_DATA = 5;
const int PID_DATA = 6;
const int INFO = 7;
const int WARNING = 8;
const int ERROR = 9;

//positionSet
//--------------------------------------------------------------------------------------------------
typedef struct {
  Point2d p; //Units
  double head; //Radians
} positionSet;

std::ostream& operator << (std::ostream& os, positionSet p);
bool operator==(const positionSet& a, const positionSet& b);
bool operator!=(const positionSet& a, const positionSet& b);

//Log
//coreLogger is implementation across libary
//--------------------------------------------------------------------------------------------------
class Log {
private:
  std::string internalString;
  
  void formatInput(std::string &s);

public:
  bool doLog = false;

  Log();
  Log(bool activeIn);
  void setActive(bool activeIn);

  void clear();
  void print(bool clearBuf=true);
  void save(const char* fileName, bool clearBuf=true);
  void append(const char* fileName, bool clearBuf=true);

  //FRAME start, FRAME end, etc
  void meta(std::string label, std::string data);
  void logTime(double t);

  void logTimeout(std::string label, int isDone, int isMoving, int atTarget, int timedOut);

  void logPoint(std::string label, Point2d p);

  void logVector(std::string label, Vector2d p);

  void logPointSet(std::string label, positionSet p);

  void logPID(std::string label, double error, double p, double i, double d);

  void status(std::string label, bool stat, int priority=GRAPH_DATA);

  void status(std::string label, double stat, int priority=GRAPH_DATA);
  void info(std::string label, std::string data);

  void warning(std::string label, std::string data);

  void error(std::string label, std::string data);
};

Log coreLogger();