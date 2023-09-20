#include "robotmath/EWMAFilter.h"

EWMAFilter::EWMAFilter(double kIn){
  setK(kIn);
}

EWMAFilter::EWMAFilter(double kIn, double initalValue){
  setK(kIn);
  lastData = initalValue;
}

void EWMAFilter::setK(double kIn){
  k = clamp(kIn, 0, 1);
}

double EWMAFilter::getK(){
  return k;
}

void EWMAFilter::setLastData(double dataIn){
  lastData = dataIn;
}

double EWMAFilter::getAvg(double dataIn){
  double r = (1-k)*lastData + k*dataIn;
  lastData = dataIn;
  return r;
}