#pragma once
#include "Point2d.h"
#include <list>

class Path {
private:
    std::list<positionSet> points;
    int index;
    Path();
public:
    int getSize() { return points.size(); }
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
    
    bool tryGetFromStart(int i, positionSet* output);
    bool tryGetFromEnd(int i, positionSet* output);

    positionSet next(bool shift = false);
    positionSet previous(bool shift = false);
    
    Path subpath(int start, int end);
    double arclength();
};