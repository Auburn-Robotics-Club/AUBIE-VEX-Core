#pragma once
#include "base.h"

class BasePIDController{
    protected: 
    PIDValues k;

    double target;
    double error;

    double setValue;

    bool initalized;
    double lastError;
    
    double minOutput; //Limits output and setvalue
    double maxOutput;

    double output;

    public:
    /**
     * Initialize a new PID Controller object.
     * @param pid_values The PID values to use for the controller.
     * @param init_target The initial target for the controller.
     * @param min_output The minimum output of the controller.
     * @param max_output The maximum output of the controller.
    */
    BasePIDController(PIDValues pid_values, double init_target, double min_output, double max_output);

    /**
     * Set a new target for the controller.
     * @param targetValue The value of the controller target.
     * @param initSetValue An optional parameter for initializing the I factor of the controller.
    */
    void setTarget(double targetValue, double initSetValue=0);

    /**
     * Retrieve the controller output value for a given input value.
     * @param currentVaule The input value for the controller.
     * @param deltaTime The time elapsed since the controller recieved its last input value
     * @return The output value of the controller.
    */
    double update(double currentValue, double deltaTime);
};