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

void Path::insert(int i, positionSet p) {
    if (i < 0) {
        return;
    }
    Node* n = front;
    while (n) {
        if (i == 0) {
            n->addBefore(Node(p));
            return;
        }
        n = n->next;
        i--;
    }
}

void Path::insert(int i, Path other) {
    if (i < 0) {
        return;
    }
    Node* n = front;
    while (n) {
        if (i == 0) {
            Node* n2 = other.front;
            while (n2) {
                n->addBefore(Node(n2->data));
                n2 = n2->next;
            }
            return;
        }
        n = n->next;
        i--;
    }
}