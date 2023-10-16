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

void Path::addToStart(Path other) {
    Node* n = other.rear;
    while (n) {
        addToStart(n->data);
        n = n->prev;
    }
}

void Path::addToEnd(Path other) {
    Node* n = other.front;
    while (n) {
        addToEnd(n->data);
        n = n->next;
    }
}