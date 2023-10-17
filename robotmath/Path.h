#pragma once
#include "Point2d.h"
#include "Vector2d.h"

class NodePS {
private:
    NodePS* next = nullptr;
    NodePS* prev = nullptr;

    friend class Node;
    friend class Path;

    NodePS* getHeapCopy(NodePS n){
        NodePS* r = new NodePS();
        r->next = n.next;
        r->prev = n.prev;
        r->data = n.data;
        return r;
    }

    NodePS(){
        data = {Point2d(0, 0), 0};
    }

    NodePS* addBefore(NodePS N){
        NodePS* n = getHeapCopy(N);
        n->prev = prev;
        n->next = this;
        if(hasPrev()){
            prev->next = n;
        }
        prev = n;
        return n;
    }

    NodePS* addAfter(NodePS N){
        NodePS* n = getHeapCopy(N);
        n->prev = this;
        n->next = next;
        if(hasNext()){
            next->prev = n;
        }
        next = n;
        return n;
    }

    void removeBefore(){
        if(!hasPrev()){
            return;
        }
        NodePS* p = prev;
        if(prev->hasPrev()){
            p->prev->next = this;
        }
        prev = p->prev;
        free(p);
    }

    void removeAfter(){
        if(!hasNext()){
            return;
        }
        NodePS* n = next;
        if(n->hasNext()){
            n->next->prev = this;
        }
        next = n->next;
        free(n);
    }
public:
    positionSet data;

    NodePS(positionSet pos){
        data = pos;
    }

    bool hasNext(){
        return !(next == nullptr);
    }

    bool hasPrev(){
        return !(prev == nullptr);
    }

    NodePS* getNext(){
        return next;
    }

    NodePS* getPrev(){
        return prev;
    }
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

    NodePS* getFront() {
        return front;
    };
    NodePS* getRear() {
        return rear;
    };

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
    TargetPath(positionSet initalPos){
        addToStart(initalPos);
    }

    void addTarget(double x, double y) {
        Point2d p = Point2d(x, y);
        addToEnd({ p, normalizeAngle(Vector2d(1, 0).getAngle(Vector2d(getRear()->data.p, p))) });
    }

    void addTarget(double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); }
        heading = normalizeAngle(heading);
        addToEnd({ getRear()->data.p, heading });
    }

    void addTarget(double x, double y, double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); };
        addToEnd({ Point2d(x, y), heading });
    }

    void addTarget(positionSet in, bool inDeg = true) {
        if (inDeg) { in.head = degToRad(in.head); }
        in.head = normalizeAngle(in.head);
        addToEnd(in);
    }

    void addTarget(Vector2d in) {
        addTarget(getRear()->data.p.x + in.getX(), getRear()->data.p.y + in.getY());
    }

    void addRelTarget(Vector2d in) {
        in = in.getRotatedVector(normalizeAngle(getRear()->data.head) - M_PI_2);
        addTarget(getRear()->data.p.x + in.getX(), getRear()->data.p.y + in.getY());
    }

    void addRelTarget(double heading, bool inDeg = true) {
        if (inDeg) { heading = degToRad(heading); }
        addToEnd({ getRear()->data.p, normalizeAngle(getRear()->data.head + heading) });
    }

    int getSize(){
        return Path::getSize();
    }

    double arclength(){
        return Path::arclength();
    }

    NodePS* getFront(){
        return Path::getFront();
    }

    NodePS* getRear() {
        return Path::getRear();
    }

    std::vector<positionSet> getList() {
        return Path::getList();
    }
};
