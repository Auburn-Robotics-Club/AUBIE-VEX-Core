#pragma once
#include "Point2d.h"
#include "Vector2d.h"

class NodePS {
private:
    NodePS* next = nullptr;
    NodePS* prev = nullptr;

    friend class Node;
    friend class Path;

    NodePS* getHeapCopy(NodePS n);

    NodePS();

    NodePS* addBefore(NodePS N);
    NodePS* addAfter(NodePS N);
    void removeBefore();
    void removeAfter();
public:
    positionSet data;

    NodePS(positionSet pos);
    bool hasNext();
    bool hasPrev();
    NodePS* getNext();
    NodePS* getPrev();
};

class Path {
private:
    NodePS* front = nullptr;
    NodePS* rear = nullptr;
    int size = 0;

public:
    Path() {};

    int getSize() { return size; }

    void addToStart(positionSet p);
    void addToEnd(positionSet p);
    void addToStart(Path otherList);
    void addToEnd(Path otherList);

    void insert(int index, positionSet set);
    void insert(int index, Path set);

    bool removeFromStart(int i);
    bool removeFromEnd(int i);
    void removeAll();
    
    NodePS* tryGetFromStart(int i);
    NodePS* tryGetFromEnd(int i);
    
    Path subpath(int start, int end);
    double arclength();

    NodePS* getFront();
    NodePS* getRear();

    //DEPRECATED!!! SIMULATOR USE ONLY
    std::vector<positionSet> getList() {
        std::vector <positionSet> result = std::vector<positionSet>();
        NodePS* n = front;
        while (n != nullptr) {
            result.push_back(n->data);
            n = n->getNext();
        }
        return result;
    }

};

class TargetPath : private Path {
public:
    //Pass in either intended start of path or current robot position
    TargetPath(positionSet initalPos);
    void addTarget(double x, double y);
    void addTarget(double heading, bool inDeg = true);
    void addTarget(double x, double y, double heading, bool inDeg = true);
    void addTarget(positionSet in, bool inDeg = true);
    void addTarget(Vector2d in);
    void addRelTarget(Vector2d in);
    void addRelTarget(double heading, bool inDeg = true);
    int getSize();
    double arclength();
    NodePS* getFront();
    NodePS* getRear();
    std::vector<positionSet> getList();
};
