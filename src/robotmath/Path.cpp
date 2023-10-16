#include "../../robotmath/Path.h"

void Path::addToStart(positionSet p){
    if(front == nullptr){
        front = new Node(p);
        rear = front;
        return;
    }

    front = front->addBefore(Node(p));
};

void Path::addToEnd(positionSet p){
    if(front == nullptr){
        addToStart(p);
        return;
    }

    rear = rear->addAfter(Node(p));
}