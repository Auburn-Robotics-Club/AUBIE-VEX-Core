#pragma once
#include "base.h"

/**
 * Class for an Exponentially Weighted Moving Average Filter.
 * Read more: https://hackaday.com/2019/09/06/sensor-filters-for-coders/
 */
class EWMAFilter{
    protected:
    double k;
    double lastData;

    public:
    /**
     * Initialize an EWMA Filter object.
     * @param kIn The weight of the filter.
    */
    EWMAFilter(double kIn);

    /**
     * Initialize an EWMA Filter object with an initial data value.
     * @param kIn The weight of the filter.
     * @param initialValue The initial value to set the filter to.
    */
    EWMAFilter(double kIn, double initalValue);

    /**
     * Set the weight of the filter.
     * The weight must be between 0 and 1 inclusive.
     * @param kIn The new weight to set the filter to.
    */
    void setK(double kIn);

    /**
     * Get the current weight of the filter.
     * @return The current weight of the filter.
    */
    double getK();

    /**
     * Manually set the most recent data value in the filter.
     * @param dataIn The data value to set in the filter.
    */
    void setLastData(double dataIn);

    /**
     * Get the filter average with the most recent data value and a new data value.
     * @param dataIn The new data value to use for the average.
     * @return The filter average with the new data value.
    */
    double getAvg(double dataIn);
};