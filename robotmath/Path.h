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

public:
    Path();

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
};