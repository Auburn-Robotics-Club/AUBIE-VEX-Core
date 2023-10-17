#pragma once
#include "robotmath.h"

class Node {
private:
    Node* next = nullptr;
    Node* prev = nullptr;

    friend Node;
    friend Path;

    Node* getHeapCopy(Node n){
        Node* r = new Node();
        r->next = n.next;
        r->prev = n.prev;
        r->data = n.data;
        return r;
    }

    Node(){
        data = {Point2d(0, 0), 0};
    }

    Node* addBefore(Node N){
        Node* n = getHeapCopy(N);
        n->prev = prev;
        n->next = this;
        if(hasPrev()){
            prev->next = n;
        }
        prev = n;
        return n;
    }

    Node* addAfter(Node N){
        Node* n = getHeapCopy(N);
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
        Node* p = prev;
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
        Node* n = next;
        if(n->hasNext()){
            n->next->prev = this;
        }
        next = n->next;
        free(n);
    }
public:
    positionSet data;

    Node(positionSet pos){
        data = pos;
    }

    bool hasNext(){
        return !(next == nullptr);
    }

    bool hasPrev(){
        return !(prev == nullptr);
    }

    Node* getNext(){
        return next;
    }

    Node* getPrev(){
        return prev;
    }
};

class Path {
private:
    Node* front = nullptr;
    Node* rear = nullptr;
    int size = 0;

public:
    Path();
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
    
    bool tryGetFromStart(int i, Node** output);
    bool tryGetFromEnd(int i, Node** output);
    
    Path subpath(int start, int end);
    double arclength();

    Node* getFront();
    Node* getRear();
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
        addTarget(in.getX(), in.getY());
    }

    void addRelTarget(Vector2d in) {
        in = in.getRotatedVector(normalizeAngle(getRear()->data.head) - M_PI_2);
        addTarget(in.getX(), in.getY());
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

    Node* getFront(){
        return Path::getFront();
    }
};