#ifndef __LOGGER_H__
#define __LOGGER_H__

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

std::string replaceStr(std::string str, const std::string& from, const std::string& to) {
  size_t start_pos = 0;
  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
  }
  return str;
}

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

typedef struct {
  Point2d p; //Units
  double head; //Radians
} positionSet;

class Log;
extern Log coreLogger;

class Log {
private:
  std::string internalString;
  
  void formatInput(std::string &s) {
    s = replaceStr(s, "\n", "");
  }

public:
  bool doLog;

  Log(bool active){
    doLog = active;
    clear();
  }

  void clear(){
    internalString.clear();
  }

  void print(bool clearBuf=true){
    if(doLog){
      std::cout << internalString << std::flush;
      if(clearBuf){
        clear();
      }
    }
  }

  void save(const char* fileName, bool clearBuf=true){
    if(doLog){
      std::ofstream file(fileName);
      if(file.is_open()){
        file.clear();
        file << internalString << std::flush;
        file.close();
      }
      if(clearBuf){
        clear();
      }
    }
  }

  void append(const char* fileName, bool clearBuf=true){
    if(doLog){
      std::ofstream file(fileName, std::ios_base::app);
      if(file.is_open()){
        file << internalString << std::flush;
        file.close();

        if(clearBuf){
          clear();
        }
      }
    }
  }

  //FRAME start, FRAME end, etc
  void meta(std::string label, std::string data){
    if(doLog){
      formatInput(label);
      formatInput(data);

      char buffer [100];
      sprintf(buffer, "%d,%s,%s\n", META_DATA, label.c_str(), data.c_str());
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logTime(double t){
    if(doLog){
      char buffer [100];
      sprintf(buffer, "%d,TIME,%.2f\n", META_DATA, t);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logTimeout(std::string label, int isDone, int isMoving, int atTarget, int timedOut){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%d,%d,%d,%d\n", TIMEOUT_DATA, label.c_str(), isDone, isMoving, atTarget, timedOut);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logPoint(std::string label, Point2d p){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%.2f,%.2f\n", POINT_DATA, label.c_str(), p.x, p.y);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logVector(std::string label, Vector2d p){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%.5f,%.5f\n", VECTOR_DATA, label.c_str(), p.getX(), p.getY());
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logPointSet(std::string label, positionSet p){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%.2f,%.2f,%.2f\n", POINTSET_DATA, label.c_str(), p.p.x, p.p.y, radToDeg(p.head));
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void logPID(std::string label, double error, double p, double i, double d){
    if(doLog){
      formatInput(label);
      char buffer [200];
      sprintf(buffer, "%d,%s,%.2f,%.2f%.2f%.2f\n", PID_DATA, label.c_str(), error, p, i, d);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void status(std::string label, bool stat, int priority=GRAPH_DATA){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%d\n", priority, label.c_str(), stat);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void status(std::string label, double stat, int priority=GRAPH_DATA){
    if(doLog){
      formatInput(label);
      char buffer [100];
      sprintf(buffer, "%d,%s,%.3f\n", priority, label.c_str(), stat);
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void info(std::string label, std::string data){
    if(doLog){
      formatInput(label);
      formatInput(data);
      char buffer [200];
      sprintf(buffer, "%d,%s,%s\n", INFO, label.c_str(), data.c_str());
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void warning(std::string label, std::string data){
    if(doLog){
      formatInput(label);
      formatInput(data);
      char buffer [200];
      sprintf(buffer, "%d,%s,%s\n", WARNING, label.c_str(), data.c_str());
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }

  void error(std::string label, std::string data){
    if(doLog){
      formatInput(label);
      formatInput(data);
      char buffer [200];
      sprintf(buffer, "%d,%s,%s\n", ERROR, label.c_str(), data.c_str());
      std::string result = std::string(buffer);
      result.shrink_to_fit();
      internalString += result;
    }
  }
};

std::ostream& operator << (std::ostream& os, Vector2d v){
  os << "<" << v.getX() << ", " << v.getY() << ">";
  return os;
}

std::ostream& operator << (std::ostream& os, Point2d p){
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

std::ostream& operator << (std::ostream& os, positionSet p){
  os << "(" << p.p.x << ", " << p.p.y << ", " << p.head << ")";
  return os;
}

/*
class PythonProgramLogger{
  protected:
  const char* graphName;
  Log logger;

  public:
  PythonProgramLogger(const char* logName) : logger(logName) {
    graphName = logName;
    save();
  }

  void graph(){
    logger << "END:END";
  }

  void addPoint(Point p, const char* color){
    logger << "P:" << p.x << "," << p.y << "," << color << std::endl;
  }

  void addVector(Point p, Vector v, const char* color){
    logger << "V:" << p.x << "," << p.y << "," << v.getX() << "," << v.getY() << "," << color << std::endl;
  }

  void save(bool clearBuffer=true){
    logger.save(clearBuffer);
  }

  void append(bool clearBuffer=true){
    logger << std::endl;
    logger.append(clearBuffer);
  }

  void print(bool clearBuffer=true){
    logger << std::endl;
    logger.print(clearBuffer);
  }
};

typedef struct{
  double error;
  PIDOutput pidOutput;
  double realTargetVel;
  double realVel;
} PIDData;

class Graph{
public:
    PIDData* linearPIDData = (PIDData*)malloc(0);
    int numOfLinPIDData = 0;
    PIDData* rotPIDData = (PIDData*)malloc(0);
    int numOfRotPIDData = 0;
    std::ofstream file;

    Graph(const char* fileName){
      file.open(fileName, std::fstream::out | std::fstream::app);
    }

    ~Graph() {
        free(linearPIDData);
        free(rotPIDData);
        file.close();
    }

    void clear() {
        free(linearPIDData);
        linearPIDData = (PIDData*)malloc(0);
        numOfLinPIDData = 0;

        free(rotPIDData);
        rotPIDData = (PIDData*)malloc(0);
        numOfRotPIDData = 0;
    }

    void addPID(PIDData data, bool linearPID) {
      if(linearPID){
        numOfLinPIDData++;
        linearPIDData = (PIDData*)realloc(linearPIDData, sizeof(PIDData) * numOfLinPIDData);
        linearPIDData[numOfLinPIDData - 1] = data;
      }else{
        numOfRotPIDData++;
        rotPIDData = (PIDData*)realloc(rotPIDData, sizeof(PIDData) * numOfRotPIDData);
        rotPIDData[numOfRotPIDData - 1] = data;
      }
    }

    std::string getString() {
        std::ostringstream strs;
        for (int i = 0; i < numOfLinPIDData; i++) {
            PIDData d = linearPIDData[i];
            strs << "LPID:" << d.error << "," << d.pidOutput.output << "," << d.pidOutput.reset << "," << d.realTargetVel << "," << d.realVel << std::endl;
        }
        for (int i = 0; i < numOfRotPIDData; i++) {
            PIDData d = rotPIDData[i];
            strs << "RPID:" << d.error << "," << d.pidOutput.output << "," << d.pidOutput.reset << "," << d.realTargetVel << "," << d.realVel << std::endl;
        }
        strs << "END:END" << std::endl;
        return strs.str();
    }
};*/

#endif