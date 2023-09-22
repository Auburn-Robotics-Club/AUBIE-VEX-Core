#include "../../robotmath/robotmath.h"

Point2d::Point2d() {
    x = 0;
    y = 0;
}

Point2d::Point2d(double X, double Y) {
  x = X;
  y = Y;
}

//Calculates midpoint of 2 points
Point2d midpoint(Point2d a, Point2d b) {
    return Point2d((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
}

Point2d operator + (Point2d p, Vector2d v) {
    return v + p;
};

std::ostream& operator << (std::ostream& os, Point2d p){
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, const std::vector<Vector2d>& vList) {
    std::vector<Point2d> result;
    if (pList.size() >= vList.size()) {
        for (int i = 0; i < vList.size(); i++) {
            result.push_back(pList[i] + vList[i]);
        }
    } else {
        for (int i = 0; i < pList.size(); i++) {
            result.push_back(pList[i] + vList[i]);
        }
    }
    return result;
};

std::vector<Point2d> operator + (const std::vector<Vector2d>& vList, const std::vector<Point2d>& pList) {
    return pList + vList;
};

std::vector<Point2d> operator + (const std::vector<Point2d>& pList, Vector2d v) {
    std::vector<Point2d> result;
    for (int i = 0; i < pList.size(); i++) {
        result.push_back(pList[i] + v);
    }
    return result;
};

std::vector<Point2d> operator * (const std::vector<Point2d>& pList, double scale) {
    std::vector<Point2d> result;
    for (int i = 1; i < pList.size(); i++) {
        result.push_back(pList[0] + (Vector2d(pList[0], pList[i]).scale(scale)));
    }
    return result;
};

std::vector<Point2d> operator || (const std::vector<Point2d>& pList, double radiansCCW) {
    std::vector<Point2d> result;
    if (pList.size() > 0) {
        result.push_back(pList[0]);
        for (int i = 1; i < pList.size(); i++) {
            result.push_back(pList[0] + (Vector2d(pList[0], pList[i]).getRotatedVector(radiansCCW)));
        }
    }
    return result;
};