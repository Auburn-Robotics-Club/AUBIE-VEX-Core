#pragma once
#include "base.h"

/**
 * Class for a Simple Moving Averge data filter.
 * Read more: https://hackaday.com/2019/09/06/sensor-filters-for-coders/
 */
class SMAFilter{
    protected:
    std::vector<double> data;

    public:
    /**
     * Initializes an SMA Filter object with values of 0.
     * @param size The size of the filter.
    */
    SMAFilter(int size);

    /**
     * Initializes an SMA Filter object with preset values.
     * @param size The size of the filter.
     * @param value The value to preset the filter with.
    */
    SMAFilter(int size, double value);

    /**
     * Changes the size of the filter. 
     * If the new size is smaller than the current size, the oldest data entries will be discarded.
     * If the new size is larger than the current size, the new slots will be filled with a preset value.
     * @param size The new size to set the filter to.
     * @param value The value to preset the filter with if the new size is larger than the current size.
    */
    void changeSize(double size, double value);

    /**
     * Sets all entries in the filter to a preset value.
     * @param value The value to set the entries to.
    */
    void clear(double value=0);

    /**
     * Add a new entry to the filter.
     * The oldest entry will be discarded.
     * @param value The value to add to the filter.
    */
    void add(double value);

    /**
     * Get the current average of the entries in the filter.
     * @return The current averge of the filter.
    */
    double getAvg();
};