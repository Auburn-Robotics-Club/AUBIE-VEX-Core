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

bool Path::removeFromStart(int i) {
    if (i < 0) {
        return false;
    }

    Node* n = front;
    while (n) {
        if (i == 0) {
            if (n == front) {
                front = n->next;
            }
            if (n->hasNext()) {
                n->next->removeBefore();
            } else { // n == rear must be true here
                rear = n->prev;
            }
            free(n);
            return true;
        }
        n = n->next;
        i--;
    }
    return false;
}

bool Path::removeFromEnd(int i) {
    if (i < 0) {
        return false;
    }
    
    Node* n = rear;
    while (n) {
        if (i == 0) {
            if (n == rear) {
                rear = n->prev;
            }
            if (n->hasPrev()) {
                n->prev->removeAfter();
            } else { // n == front must be true here
                front = n->next;
            }
            return true;
        }
        n = n->prev;
        i--;
    }
    return false;
}

void Path::removeAll() {
    while (front->hasNext()) {
        front->removeAfter();
    }
    free(front);
    front = rear = nullptr;
}