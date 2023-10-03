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

    size--; // Removing at least 1 node if we get here

    PathNode* n = this->start;
    while (i > 0) {
        if (!n) {
            break;
        }
        n = n->next;
        i--;
        size--;
    }

    PathNode* temp = n->previous;
    while (temp) {
        PathNode* temp2 = temp;
        temp = temp->previous;
        delete temp2;
    }
    delete temp;

    n->previous = nullptr;
    this->start = n;

    return true;
}