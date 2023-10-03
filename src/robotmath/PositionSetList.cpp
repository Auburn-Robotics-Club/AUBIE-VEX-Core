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

bool Path::removeFromEnd(int i) {
    if (i < 0) {
        return false;
    }
    if (i >= size) {
        this->removeAll();
        return true;
    }

    PathNode* n;
    this->tryGetFromEnd(i, &n); // This will never fail because 0 <= i < size
    
    this->end = n->previous;
    while (n) {
        PathNode* temp = n;
        n = n->next;
        delete temp;
    }

    size -= i + 1;

    return true;
}

void Path::removeAll() {
    PathNode* n = this->start;
    while (n) {
        PathNode* temp = n;
        n = n->next;
        delete n;
    }
    this->start = this->end = nullptr;
    size = 0;
}

bool Path::tryGetFromStart(int i, PathNode** out) {
    *out = nullptr;

    if (i < 0 || i >= size) {
        return false;
    }

    PathNode* n = this->start;
    while (i > 0) {
        n = n->next;
        i--;
    }

    if (!n) {
        return false;
    }
    
    *out = n;
    return true;
}

bool Path::tryGetFromEnd(int i, PathNode** out) {
    *out = nullptr;

    if (i < 0 || i >= size) {
        return false;
    }

    PathNode* n = this->end;
    while (i > 0) {
        n = n->previous;
        i--;
    }
    
    if (!n) {
        return false;
    }

    *out = n;
    return true;
}

double Path::arclengthFromStart(int i, int count) {
    if (count < 1) {
        return 0;
    }

    PathNode* prev;
    if (!this->tryGetFromStart(i, &prev)) {
        return 0;
    }

    double sum = 0;
    PathNode* curr = prev->next;
    while (count > 0 && curr) {
        double dx = prev->pose.p.x - curr->pose.p.x;
        double dy = prev->pose.p.y - curr->pose.p.y;
        sum += sqrtf((dx * dx) + (dy * dy));

        prev = curr;
        curr = curr->next;
        count--;
    }

    return sum;
}

double Path::arclengthFromEnd(int i, int count) {
    if (count < 1) {
        return 0;
    }

    PathNode* prev;
    if (!this->tryGetFromEnd(i, &prev)) {
        return 0;
    }

    double sum = 0;
    PathNode* curr = prev->previous;
    while (count > 0 && curr) {
        double dx = prev->pose.p.x - curr->pose.p.x;
        double dy = prev->pose.p.y - curr->pose.p.y;
        sum += sqrtf((dx * dx) + (dy * dy));

        prev = curr;
        curr = curr->previous;
        count--;
    }

    return sum;
}