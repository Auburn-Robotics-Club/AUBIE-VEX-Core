#pragma once
#include "Point2d.h"

class PathNode {
public:
    positionSet pose;
    PathNode* previous;
    PathNode* next;
    PathNode* insert(positionSet set);
    PathNode* insert(Path list);
};

class Path {
private:
    PathNode* end;
public:
    PathNode* getEnd() { return end; }
    void add(positionSet set);
    void add(Path otherList);
    bool remove(int i);
    void clear();
    bool tryGet(int i, PathNode** output);
    bool tryGetRelative(int current, int offset, PathNode** output);
    double arclength(int start, int count);
    int size();
};