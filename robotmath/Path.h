#pragma once
#include "Point2d.h"

class PathNode {
public:
    positionSet pose;
    Path* path;
    PathNode* previous;
    PathNode* next;
};

class Path {
private:
    PathNode* start;
    PathNode* end;
    int size;
public:
    PathNode* getStart() { return start; }
    PathNode* getEnd() { return end; }
    int getSize() { return size; }

    void addToStart(positionSet set);
    void addToStart(Path otherList);
    void addToEnd(positionSet set);
    void addToEnd(Path otherList);

    bool removeFromStart(int i);
    bool removeFromEnd(int i);
    void removeAll();
    
    bool tryGetFromStart(int i, PathNode** output);
    bool tryGetFromEnd(int i, PathNode** output);
    
    double arclengthFromStart(int i, int count);
    double arclengthFromEnd(int i, int count);
};