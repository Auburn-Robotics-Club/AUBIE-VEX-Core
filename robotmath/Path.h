#pragma once
#include "robotmath.h"

class Node {
private:
    Node* next = nullptr;
    Node* prev = nullptr;

    friend Node;

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
};

class NodeList {
private:
    Node* front = nullptr;
    Node* rear = nullptr;

public:
    void addToStart(positionSet p){
        if(front == nullptr){
            front = new Node(p);
            rear = front;
            return;
        }

        front = front->addBefore(Node(p));
    };
    void addToEnd(positionSet p){
        if(front == nullptr){
            addToStart(p);
            return;
        }

        rear = rear->addAfter(Node(p));
    };
};

class Path {
private:
    std::list<positionSet> points;
    int index;
public:
    Path();
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

    positionSet get();
    positionSet getReference();
    positionSet next(bool shift = false);
    positionSet previous(bool shift = false);
    
    Path subpath(int start, int end);
    double arclength();
};