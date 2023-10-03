#pragma once
#include "Point2d.h"

class Path {
public:
    positionSet pose;
    Path* previous;
    Path* next;
    Path* insert(positionSet set);
    Path* insert(PathNode list);
};

class PathNode {
private:
    Path* end;
public:
    Path* getEnd() { return end; }
    void add(positionSet set);
    void add(PathNode otherList);
    bool remove(int i);
    void clear();
    bool tryGet(int i, Path** output);
    bool tryGetRelative(int current, int offset, Path** output);
    double arclength(int start, int count);
    int size();
};