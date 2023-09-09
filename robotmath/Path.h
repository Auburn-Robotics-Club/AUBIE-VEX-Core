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
         * Initialize a new Path object with a group of positionSets.
         * 
         * @param pointsIn A vector of positionSets to be used for the path.
        */
        Path(std::vector<positionSet> &pointsIn);

        /**
         * Retrieve the positionSets used for the path.
         * 
         * @return A vector of the positionSets in the path.
        */
        std::vector<positionSet>& getList();

        /**
         * Manually set the path index.
         * The index must be between 0 and the path size - 1 inclusive.
         * 
         * @param i The new path index (0 by default).
        */
        void setIndex(int i = 0);

        /**
         * Retrieve the path size (the number of positionSets in the path).
         * 
         * @return The path size.
        */
        int size();

        /**
         * Retrive the next positionSet in the path.
         * If the path has no more positionSets, the positionSet {Point2d(0, 0), 0} is returned.
         * 
         * @return A positionSet object of the next positionSet in the path.
        */
        positionSet next();

        /**
         * Returns whether the path has more positionSets remaining.
         * 
         * @return A boolean of whether the path has more positionSets remaining.
        */
        bool hasNext();

        /**
         * Get the current path index.
         * 
         * @return The current path index.
        */
        int index();

        /**
         * Get the positionSet at a specific index in the path.
         * 
         * @param index The index of the positionSet to retrieve.
         * 
         * @return The positionSet object at the index i.
        */
        positionSet get(int index);

        /**
         * Erases all positionSets in the path up to a specific index.
         * 
         * @param index The index to erase all positionSets up to.
        */
        void drop(int index);

        /**
         * Clears all positionSets from the path.
        */
        void clear();

        /**
         * Add a new postionSet to the path.
         * 
         * @param pS The positionSet to add.
        */
        void addPointset(positionSet pS);

        /**
         * Add a new postionSet to the path.
         * 
         * @param point The point for the positionSet.
         * @param head The head for the positionSet.
         * @param inDeg A boolean of whether the head is in degrees (true by default).
        */
        void addPointset(Point2d point, double head, bool inDeg = true);

        /**
         * Get the arc length of the path from a start index to an end index.
         * 
         * @param start The start index to use (0 by default).
         * @param end The end index to use (end of path by default).
         * 
         * @return The arc length of the segment of the path.
        */
        double arclength(int start = 0, int end = -1);

        /**
         * Get the arc length of the path from the current index to an end index.
         * 
         * @param end The end index to use (end of path by default).
         * 
         * @return The arc length of the segment of the path.
        */
        double arclengthFromIndexTo(int end = -1);

        /**
         * Add a portion of this path to another path.
         * 
         * @param pathIn The path to add the segment to.
         * @param start The start index of the path segment.
         * @param end The end index of the path segment.
        */
        void subpath(Path& pathIn, int start, int end);

        /**
         * Get a segment of this path as a new path.
         * 
         * @param start The start index of the path segment.
         * @param end The end index of the path segment.
         * 
         * @return A new Path object representing the segment.
        */
        Path subpath(int start, int end);

        //Append
        positionSet operator [] (int index);
        Path& operator + (positionSet& p);
        Path& operator + (Path& p);
};

std::ostream& operator << (std::ostream& os, Path& p);

#endif