#pragma once
#include "Point2d.h"

class PostionSetListNode {
public:
    positionSet pose;
    PostionSetListNode* previous;
    PostionSetListNode* next;
    PostionSetListNode* insert(positionSet set);
    PostionSetListNode* insert(PositionSetList list);
};

class PositionSetList {
private:
    PostionSetListNode* end;
public:
    PostionSetListNode* getEnd() { return end; }
    void add(positionSet set);
    void add(PositionSetList otherList);
    bool remove(int i);
    void clear();
    bool tryGet(int i, PostionSetListNode** output);
    bool tryGetRelative(int current, int offset, PostionSetListNode** output);
    double arclength(int start, int count);
    int size();
};