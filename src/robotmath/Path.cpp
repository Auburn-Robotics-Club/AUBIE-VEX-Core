#include "../../robotmath/Path.h"

void Path::addToStart(positionSet p) {
    size++;
    if (front == nullptr) {
        front = new NodePS(p);
        rear = front;
        return;
    }

    front = front->addBefore(NodePS(p));
};

void Path::addToEnd(positionSet p) {
    size++;
    if (front == nullptr) {
        addToStart(p);
        return;
    }

    rear = rear->addAfter(NodePS(p));
}

void Path::addToStart(Path other) {
    NodePS* n = other.rear;
    while (n) {
        size++;
        addToStart(n->data);
        n = n->prev;
    }
}

void Path::addToEnd(Path other) {
    NodePS* n = other.front;
    while (n) {
        size++;
        addToEnd(n->data);
        n = n->next;
    }
}

void Path::insert(int i, positionSet p) {
    if (i < 0) {
        return;
    }
    NodePS* n = front;
    while (n) {
        if (i == 0) {
            size++;
            n->addBefore(NodePS(p));
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
    NodePS* n = front;
    while (n) {
        if (i == 0) {
            NodePS* n2 = other.front;
            while (n2) {
                size++;
                n->addBefore(NodePS(n2->data));
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

    NodePS* n = front;
    while (n) {
        if (i == 0) {
            if (n == front) {
                front = n->next;
            }
            if (n->hasNext()) {
                n->next->removeBefore();
            }
            else { // n == rear must be true here
                rear = n->prev;
            }
            free(n);
            size--;
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

    NodePS* n = rear;
    while (n) {
        if (i == 0) {
            if (n == rear) {
                rear = n->prev;
            }
            if (n->hasPrev()) {
                n->prev->removeAfter();
            }
            else { // n == front must be true here
                front = n->next;
            }
            size--;
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
    size = 0;
}

NodePS* Path::tryGetFromStart(int i) {
    if (i < 0) {
        return nullptr;
    }
    NodePS* n = front;
    while (n) {
        if (i == 0) {
            return n;
        }
        n = n->next;
        i--;
    }
    return nullptr;
}

NodePS* Path::tryGetFromEnd(int i) {
    if (i < 0) {
        return nullptr;
    }
    NodePS* n = rear;
    while (n != nullptr) {
        if (i == 0) {
            return n;
        }
        n = n->prev;
        i--;
    }
    return nullptr;
}

Path Path::subpath(int start, int end) {
    start = max(0, start);
    if (start > end) {
        return Path();
    }

    NodePS* n = front;
    for (int i = 0; i < start; i++) {
        if (!n) {
            return Path();
        }
        n = n->next;
    }

    Path p = Path();

    p.size = end - start + 1;
    for (int i = 0; i < p.size; i++) {
        if (!n) {
            break;
        }
        p.addToEnd(n->data);
        n = n->next;
    }

    return p;
}

double Path::arclength() {
    if (!front) {
        return 0;
    }
    double sum = 0;
    NodePS* n = front->next;
    while (n) {
        double dx = n->data.p.x - n->prev->data.p.x;
        double dy = n->data.p.y - n->prev->data.p.y;

        sum += sqrtf((dx * dx) + (dy * dy));

        n = n->next;
    }
    return sum;
}