#include "../../robotmath/robotmath.h"

SMAFilter::SMAFilter(int size) { 
  changeSize(size, 0); 
}
SMAFilter::SMAFilter(int size, double value) { 
  data.resize(size, value); 
}

void SMAFilter::changeSize(double size, double value=0) {
  if (size < data.size()) {
    rotate(data.begin(), data.begin() + data.size() - size, data.end());
  }
  data.resize(size, value);
}

void SMAFilter::clear(double value) {
  for (int i = 0; i < data.size(); i++) {
    data[i] = value;
  }
}

void SMAFilter::add(double value) {
  rotate(data.begin(), data.begin() + 1, data.end()); // Shift Left
  data.back() = value; // Replace last element with new data
}

double SMAFilter::getAvg() {
  double sum = 0;
  int s = data.size();
  for (int i = 0; i < s; i++) {
    sum += data[i];
  }
  return sum / s;
}