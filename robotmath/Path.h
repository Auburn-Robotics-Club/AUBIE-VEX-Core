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
    int index;
    void copyNodes();
public:
    PathNode* getStart() { return start; }
    PathNode* getEnd() { return end; }
    int getSize() { return size; }
    int getIndex() { return index; }

    void addToStart(positionSet set);
    void addToStart(Path otherList);
    void addToEnd(positionSet set);
    void addToEnd(Path otherList);
    void insert(int index, positionSet set);
    void insert(int index, Path set);

    bool removeFromStart(int i);
    bool removeFromEnd(int i);
    void removeAll();
    
    bool tryGetFromStart(int i, PathNode** output);
    bool tryGetFromEnd(int i, PathNode** output);

    positionSet next(bool shift = false);
    positionSet previous(bool shift = false);
    
    Path subpath(int start, int end);
    double arclength();
};