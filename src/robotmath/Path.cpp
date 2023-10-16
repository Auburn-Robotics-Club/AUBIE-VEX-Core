#include "../../robotmath/Path.h"

Path::Path() {
    this->points = std::list<positionSet>();
}

void Path::addToStart(positionSet set) {
    this->points.push_front(set);
}

void Path::addToStart(Path path) {
    this->points.insert(this->points.begin(), path.points.begin(), path.points.end());
}

void Path::addToEnd(positionSet set) {
    this->points.push_back(set);
}

void Path::addToEnd(Path path) {
    this->points.insert(this->points.end(), path.points.begin(), path.points.end());
}

void Path::insert(int i, positionSet set) {
    if (i < 0 || i >= this->getSize()) {
        return;
    }
    auto iter = this->points.begin();
    std::advance(iter, i);
    this->points.insert(iter, 1, set);
}

void Path::insert(int i, Path path) {
    if (i < 0 || i >= this->getSize()) {
        return;
    }
    auto iter = this->points.begin();
    std::advance(iter, i);
    this->points.insert(iter, path.points.begin(), path.points.end());
}

bool Path::removeFromStart(int i) {
    if (i >= this->getSize()) {
        return false;
    }

    if (i < 0) {
        this->removeAll();
        return true;
    }

    auto iter = this->points.begin();
    std::advance(iter, i);
    this->points.erase(iter, this->points.end());
    return true;
}

bool Path::removeFromEnd(int i) {
    return this->removeFromStart(this->getSize() - 1 - i);
}

void Path::removeAll() {
    this->points.erase(this->points.begin(), this->points.end());
}

bool Path::tryGetFromStart(int i, positionSet* set) {
    set = 0;
    if (i < 0 || i >= this->getSize()) {
        return false;
    }
    auto iter = this->points.begin();
    std::advance(iter, i);
    set = &*iter;
    return true;
}

bool Path::tryGetFromEnd(int i, positionSet* set) {
    return tryGetFromStart(this->getSize() - 1 - i, set);
}

positionSet Path::get(){
    positionSet set;
    if (!tryGetFromStart(this->index, &set)) {
        return {};
    }
    return set;
}

positionSet Path::next(bool shift) {
    positionSet set;
    if (!tryGetFromStart(this->index + 1, &set)) {
        return {};
    }
    if (shift) {
        this->index++;
    }
    return set;
}

positionSet Path::previous(bool shift) {
    positionSet set;
    if (!tryGetFromStart(this->index - 1, &set)) {
        return {};
    }
    if (shift) {
        this->index++;
    }
    return set;
}

Path Path::subpath(int start, int end) {
    Path p = Path();
    if (start >= end || this->getSize() < 1) {
        return p;
    }

    auto iter = this->points.begin();
    std::advance(iter, start);
    int count = end - start;
    for (size_t i = 0; i < count; i++)
    {
        p.points.push_back(*iter);
    }    

    return p;
}

double Path::arclength() {
    if (this->getSize() < 2) {
        return 0;
    }
    double sum = 0;
    auto iter = this->points.begin();
    positionSet prev = *iter;
    iter++;
    for (int i = 1; i < this->getSize(); i++) {
        positionSet curr = *iter;

        double dx = curr.p.x - prev.p.x;
        double dy = curr.p.y - prev.p.y;

        sum += sqrtf((dx * dx) + (dy * dy));

        prev = *iter;
        iter++;
    }
    return sum;
}