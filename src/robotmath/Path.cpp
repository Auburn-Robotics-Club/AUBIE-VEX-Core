#include "robotmath.h"

Path::Path() {
    //STUB
};

Path::Path(std::vector<positionSet> &pointsIn) {
    for (int i = 0; i < pointsIn.size(); i++) {
        addPointset(pointsIn[i]);
    }
};

std::vector<positionSet>& Path::getList() {
    return points;
};

void Path::setIndex(int i) {
    if (abs(i) >= size()) {
        i = sign(i) * (size() - 1);
    }
    if (i < 0) {
        i = size() + i;
    }
    internalIndex = i;
};

int Path::size() {
    return points.size();
};

positionSet Path::next() {
    if (hasNext()) {
        internalIndex++;
        return get(internalIndex);
    }
    return { Point2d(0, 0), 0 };
};

bool Path::hasNext() {
    if (internalIndex < size()) {
        return true;
    }
    return false;
};

int Path::index() {
    return internalIndex;
};

positionSet Path::get(int i) {
    if (i >= 0) {
        if (i < size()) {
            return points[i];
        }
        else {
            return points[size() - 1];
        }
    }
    else {
        if (abs(i) > size()) { 
            return points[0];
        } else {
            return points[size() + i];
        }
    }
};

void Path::drop(int x) {
    if (x >= size()) {
        clear();
        return;
    }

    points.erase(points.begin(), points.begin() + x);
    points.shrink_to_fit();
    if (internalIndex > 0) {
        internalIndex -= x;
    }
};

void Path::clear() {
    points.clear();
    internalIndex = 0;
}

void Path::addPointset(positionSet p) {
    points.push_back(p);
};

void Path::addPointset(Point2d p, double head, bool inDeg) {
    if (inDeg) { degToRad(head); }
    points.push_back({ p, head });
};


double Path::arclength(int start, int end) {
    int si = size();
    if (start < 0) {
        if (abs(start) < si) {
            start = si + start;
        }
        else {
            start = 0;
        }
    }
    if (end < 0) {
        if (abs(end) < si) {
            end = si + end;
        }
        else {
            end = 0;
        }
    }
    if (start >= si) { start = si - 1; }
    if (end >= si) { end = si - 1; }

    double result = 0;
    for (int i = start; i <= end - 1; i++) {
        result += Vector2d(points[i].p, points[i + 1].p).getMagnitude();
    }
    return result;
};

double Path::arclengthFromIndexTo(int end) {
    return arclength(internalIndex, end);
};

void Path::subpath(Path& pathIn, int start, int end) {
    for (int i = start; i < end; i++) {
        pathIn.addPointset(get(i));
    }
};

Path Path::subpath(int start, int end) {
    Path result;
    subpath(result, start, end);
    return result;
};

positionSet Path::operator [] (int i) {
    return get(i);
};

Path& Path::operator + (positionSet& p) {
    addPointset(p);
    return *this;
};

Path& Path::operator + (Path& p) {
    for (int i = 0; i < p.size(); i++) {
        addPointset(p[i]);
    }
    return *this;
};

std::ostream& operator << (std::ostream& os, Path& p) {
    os << "{";
    for (int i = 0; i < p.size() - 1; i++) {
        os << p.get(i) << ", ";
    }
    if (p.size() > 0) {
        os << p.get(-1);
    }
    os << "}";
    return os;
};