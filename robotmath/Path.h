#ifndef PATH_H
#define PATH_H

#include <vector>
#include <sstream>
#include "mathfeeder.h"
#include "Point2d.h"

/**
 * Class for a path of positionSets.
*/
class Path {
    protected:
        int internalIndex = 0;
        std::vector<positionSet> points;

    public:
        /**
         * Initialize a new Path object.
        */
        Path();

        /**
         * Initialize a new Path object
        */
        Path(std::vector<positionSet> &pointsIn);

        std::vector<positionSet>& getList();
        void setIndex(int i = 0);
        int size();
        positionSet next();
        bool hasNext();
        int index();
        positionSet get(int i);
        void drop(int x);
        void clear();

        void addPointset(positionSet Point);
        void addPointset(Point2d p, double head, bool inDeg = true);

        double arclength(int start = 0, int end = -1);
        double arclengthFromIndexTo(int end = -1);

        void subpath(Path& pathIn, int start, int end);
        Path subpath(int start, int end);

        //Append
        positionSet operator [] (int index);
        Path& operator + (positionSet& p);
        Path& operator + (Path& p);
};

std::ostream& operator << (std::ostream& os, Path& p);

#endif