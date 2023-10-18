#include "../../robotmath/Path.h"

NodePS* NodePS::getHeapCopy(NodePS n) {
    NodePS* r = new NodePS();
    r->next = n.next;
    r->prev = n.prev;
    r->data = n.data;
    return r;
}

NodePS::NodePS() {
    data = { Point2d(0, 0), 0 };
}

NodePS* NodePS::addBefore(NodePS N) {
    NodePS* n = getHeapCopy(N);
    n->prev = prev;
    n->next = this;
    if (hasPrev()) {
        prev->next = n;
    }
    prev = n;
    return n;
}

NodePS* NodePS::addAfter(NodePS N) {
    NodePS* n = getHeapCopy(N);
    n->prev = this;
    n->next = next;
    if (hasNext()) {
        next->prev = n;
    }
    next = n;
    return n;
}

void NodePS::removeThis() {
    if (hasPrev()) {
        getPrev()->next = next;
    }
    if (hasNext()) {
        getNext()->prev = prev;
    }
    free(this);
}

void NodePS::removeBefore() {
    if (!hasPrev()) {
        return;
    }
    NodePS* p = prev;
    if (prev->hasPrev()) {
        p->prev->next = this;
    }
    prev = p->prev;
    free(p);
}

void NodePS::removeAfter() {
    if (!hasNext()) {
        return;
    }
    NodePS* n = next;
    if (n->hasNext()) {
        n->next->prev = this;
    }
    next = n->next;
    free(n);
}

NodePS::NodePS(positionSet pos) {
    data = pos;
}

bool NodePS::hasNext() {
    return !(next == nullptr);
}

bool NodePS::hasPrev() {
    return !(prev == nullptr);
}

NodePS* NodePS::getNext() {
    return next;
}

NodePS* NodePS::getPrev() {
    return prev;
}

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
                front = n->getNext();
            }
            if (n == rear) {
                rear = n->getPrev();
            }
            n->removeThis();
            size--;
            return true;
        }
        n = n->getNext();
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
            if (n == front) {
                front = n->getNext();
            }
            if (n == rear) {
                rear = n->getPrev();
            }
            n->removeThis();
            size--;
            return true;
        }
        n = n->getPrev();
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

NodePS* Path::getFront() {
    return front;
};

NodePS* Path::getRear() {
    return rear;
};

//Pass in either intended start of path or current robot position
TargetPath::TargetPath(positionSet initalPos) {
    addToStart(initalPos);
    currentNode = getFront();
}

void TargetPath::addTarget(double x, double y) {
    Point2d p = Point2d(x, y);
    addToEnd({ p, normalizeAngle(Vector2d(1, 0).getAngle(Vector2d(getRear()->data.p, p))) });
}

void TargetPath::addTarget(double heading, bool inDeg) {
    if (inDeg) { heading = degToRad(heading); }
    heading = normalizeAngle(heading);
    addToEnd({ getRear()->data.p, heading });
}

void TargetPath::addTarget(double x, double y, double heading, bool inDeg) {
    if (inDeg) { heading = degToRad(heading); };
    addToEnd({ Point2d(x, y), heading });
}

void TargetPath::addTarget(positionSet in, bool inDeg) {
    if (inDeg) { in.head = degToRad(in.head); }
    in.head = normalizeAngle(in.head);
    addToEnd(in);
}

void TargetPath::addTarget(Vector2d in) {
    addTarget(getRear()->data.p.x + in.getX(), getRear()->data.p.y + in.getY());
}

void TargetPath::addRelTarget(Vector2d in) {
    in = in.getRotatedVector(normalizeAngle(getRear()->data.head) - M_PI_2);
    addTarget(getRear()->data.p.x + in.getX(), getRear()->data.p.y + in.getY());
}

void TargetPath::addRelTarget(double heading, bool inDeg) {
    if (inDeg) { heading = degToRad(heading); }
    addToEnd({ getRear()->data.p, normalizeAngle(getRear()->data.head + heading) });
}

std::vector<positionSet> TargetPath::getList() {
    return Path::getList();
}