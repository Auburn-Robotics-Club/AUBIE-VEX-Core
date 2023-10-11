#include "../../robotmath/Path.h"
#include <iostream> // TODO: remove

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

void Path::insert(int index, positionSet set) {
    if (index < 0) {
        return;
    }
    if (index >= this->size) {
        index = this->size - 1;
    }
    PathNode* n = this->start;
    for (int i = 0; i < index; i++) {
        n = n->next;
    }
    PathNode* newNode = new PathNode();
    newNode->pose = set;
    newNode->previous = n;
    std::cout << "Path::insert > n == null > " << !n << std::endl;
    newNode->next = n->next;
    if (newNode->next) {
        newNode->next->previous = newNode;
    }
    n->next = n;
    this->size++;
}

void Path::insert(int index, Path path) {
    PathNode* other = path.start;
    while (other) {
        this->insert(index, other->pose);
        other = other->next;
        index++;
    }
}

bool Path::removeFromStart(int i) {
    std::cout << "Path::removeFromStart(" << i << ")" << std::endl;
    if (i < 0) {
        std::cout << "Path::removeFromStart > ERR > i < 0" << std::endl;
        return false;
    }
    if (i == 0) {
        this->removeAll();
        return true;
    }
    if (size == 0) {
        std::cout << "Path::removeFromStart > ERR > size == 0" << std::endl;
        return false;
    }

    PathNode* n;
    this->tryGetFromStart(i, &n); // This will never fail because 0 <= i < size
    std::cout << "Path::removeFromStart > n == null > " << !n << std::endl;

    while (n) {
        PathNode* temp = n;
        n = n->next;
        delete temp;
    }

    size -= i + 1;
    std::cout << "Path::removeFromStart > new size > " << size << std::endl;

    if (size == 1) {
        this->end = this->start;
    }

    std::cout << "Path::removeFromStart > SUCCESS!" << std::endl;
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
    std::cout << "Path::removeFromEnd > n == null > " << !n << std::endl;

    if (i == size - 1) {
        this->end = n->previous;
    }
    while (n) {
        PathNode* temp = n;
        n = n->previous;
        delete temp;
    }

    size -= i + 1;

    if (size == 1) {
        this->start = this->end;
    }

    return true;
}

void Path::removeAll() {
    PathNode* n = this->start;
    while (n) {
        PathNode* temp = n;
        n = n->next;
        delete temp;
    }
    this->start = this->end = nullptr;
    size = 0;
}

bool Path::tryGetFromStart(int i, PathNode** out) {
    *out = nullptr;

    if (i < 0 || i >= size || size == 0) {
        return false;
    }

    PathNode* n = this->start;
    while (i > 0) {
        n = n->next;
        if (!n) {
            return false;
        }
        i--;
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
        if (!n) {
            return false;
        }
        i--;
    }

    *out = n;
    return true;
}

double Path::arclength() {
    if (size < 2) {
        return 0;
    }

    double sum = 0;
    PathNode* prev = this->start;
    PathNode* curr = this->start->next;
    while (curr) {
        double dx = prev->pose.p.x - curr->pose.p.x;
        double dy = prev->pose.p.y - curr->pose.p.y;
        sum += sqrtf((dx * dx) + (dy * dy));

        prev = curr;
        curr = curr->previous;
    }

    return sum;
}

Path Path::subpath(int s, int e) {
    Path newPath;
    newPath.start = newPath.end = nullptr;
    newPath.size = newPath.index = 0;
    if (size == 0 || s >= e || s >= size || e < 0) {
        return newPath;
    }

    if (s < 0) {
        s = 0;
    }
    if (e >= size) {
        e = size - 1;
    }

    newPath.index = this->index - s;

    PathNode* newStart = this->start; 
    for (int i = 0; i < s; i++) {
        if (!newStart->next) {
            break;
        }
        newStart = newStart->next;
    }
    
    PathNode* newEnd = newStart;
    int count = end - start;
    for (int i = 0; i < count; i++) {
        if (!newEnd->next) {
            break;
        }
        newEnd = newEnd->next;
        newPath.size++;
    }

    newPath.start = newStart;
    newPath.end = newEnd;
    newPath.copyNodes();

    return newPath;
}

void Path::copyNodes() {
    PathNode* oldNode = this->start;
    PathNode* prev = nullptr;
    while (oldNode) {
        PathNode* n = new PathNode();
        n->pose = oldNode->pose;
        n->previous = prev;
        if (prev) {
            prev->next = n;
        } else {
            this->start = prev;
        }
        prev = n;
        oldNode = oldNode->next;
    }
    this->end = prev;
}

positionSet Path::next(bool shift) {
    PathNode* node;
    int i = min(this->index + 1, size - 1);
    if (!this->tryGetFromStart(i, &node)) {
        return {};
    }
    if (shift) {
        this->index = i;
    }
    std::cout << "Path::next > n == null > " << !node << std::endl;
    return node->pose;
}

positionSet Path::previous(bool shift) {
    PathNode* node;
    int i = max(this->index - 1, 0);
    if (!this->tryGetFromStart(i, &node)) {
        return {};
    }
    if (shift) {
        this->index = i;
    }
    std::cout << "Path::next > n == null > " << !node << std::endl;
    return node->pose;
}