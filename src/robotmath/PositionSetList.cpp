#include "Path.h"

void Path::addToStart(positionSet pose) {
    PathNode* n = new PathNode();
    n->path = this;
    n->pose = pose;
    n->previous = nullptr;
    n->next = this->start;

    this->start = n;
    if (size == 0) {
        this->end = n;
    }

    size++;
}

void Path::addToEnd(positionSet pose) {
    PathNode* n = new PathNode();
    n->path = this;
    n->pose = pose;
    n->previous = this->end;
    n->next = nullptr;

    this->end = n;
    if (size == 0) {
        this->start = n;
    }

    size++;
}

void Path::addToStart(Path other) {
    PathNode* n = other.end;
    while (n) {
        this->addToStart(n->pose);
        n = n->previous;
    }
}

void Path::addToEnd(Path other) {
    PathNode* n = other.start;
    while (n) {
        this->addToEnd(n->pose);
        n = n->next;
    }
}

bool Path::removeFromStart(int i) {
    if (i < 0) {
        return false;
    }
    if (i >= size) {
        this->removeAll();
        return true;
    }

    PathNode* n;
    this->tryGetFromStart(i, &n); // This will never fail because 0 <= i < size
    
    this->start = n->next;
    while (n) {
        PathNode* temp = n;
        n = n->previous;
        delete temp;
    }

    size -= i + 1;

    return true;
}