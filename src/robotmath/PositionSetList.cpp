#include "PositionSetList.h"

Path* Path::insert(PathNode list) {
    Path* n = list.getEnd();
    if (!n) {
        return this;
    }

    // Find start of list
    while (n->previous) {
        n = n->previous;
    }

    // Add self before the start of the list
    n->previous = this;
    this->next = n;
    return list.getEnd();
}

Path* Path::insert(positionSet set) {
    Path* n = new Path();
    n->pose = set;
    n->previous = this;
    this->next = n;
    return n;
}

void PathNode::add(positionSet set) {
    if (!this->end) {
        Path* n = new Path();
        n->pose = set;
        //n->previous = nullptr;
        //n->next = nullptr;
        this->end = n;
        return;
    }

    this->end = this->end->insert(set);
}

void PathNode::add(PathNode other) {
    this->end = this->end->insert(other);
}

static void deleteNode(Path* node) {
    if (!node) {
        return;
    }
    if (node->previous) {
        deleteNode(node->previous);
    }
    delete node;
}

bool PathNode::remove(int index) {
    if (index < 0) {
        return false;
    }

    if (index == 0) {
        if (this->end) {
            this->end = nullptr;
            deleteNode(this->end);
            return true;
        }
        return false;
    }

    Path* n = this->end;
    Path* beforeN = nullptr;
    while (index > 0) {
        beforeN = n;
        n = n->previous;
        if (!n) {
            return false;
        }
        index--;
    }
    beforeN->previous = nullptr;
    deleteNode(n);
    return false;
}

void PathNode::clear() {
    remove(0);
}

bool PathNode::tryGet(int i, Path** out) {
    if (i < 0) {
        return false;
    }
    Path* node = this->end;
    while (i > 0 && node) {
        node = node->previous;
        i--;
    }

    if (node) {
        if (out) {
            *out = node;
        }
        return true;
    }
    return false;
}

bool PathNode::tryGetRelative(int current, int offset, Path** out) {
    return tryGet(current + offset, out);
}

int PathNode::size() {
    int count = 0;
    Path* node = this->end;
    while (node) {
        node = node->previous;
        count++;
    }
    return count;
}

double PathNode::arclength(int start, int count) {
    if (start < 0 || count == 0) {
        return 0;
    }
    // size == 0 || size == 1
    if (!this->end || !this->end->previous) {
        return 0;
    }

    Path* prev = nullptr;
    Path* curr = this->end;

    while (start > 0) {       
        prev = curr;
        curr = curr->previous;

        if (!curr) {
            return 0;
        }
        
        start--;
    }
    
    double sum = 0;
    // If count < 0 then this will iter to the end of the list
    // If count > 0 then this will iter count times
    while (count != 0) {
        prev = curr;
        curr = curr->previous;
        if (!curr) {
            break;
        }

        double dx = prev->pose.p.x - curr->pose.p.x;
        double dy = prev->pose.p.y - curr->pose.p.y;
        sum += sqrtl((dx * dx) + (dy * dy));

        count--;
    }

    return sum;
}