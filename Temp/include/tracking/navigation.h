#pragma once

#include "robotmath.h"
#include "logger.h"

class Path{
protected:
  size_t index = 0;
  std::vector<positionSet> points;

public:
  const std::vector<positionSet> * const getList();
  void setIndex(size_t newIndex=0); //If index > list.size set to list.size
  size_t size();
  bool hasNext();
  positionSet get();
  void drop(); //Delete upto and including current index then set index to 0

  void addPointset(positionSet Point);
  
  //For target:
  //Arclength for total and remaining path, generate points, read from file, event points, nextPoint
  //Given set of points interpolate
  
  //Smarter navigation
  //Store last couple points to measure derivitives
  //Predicitive motion
  //Extrapolate from points
};

class Tracking{
  //
};
