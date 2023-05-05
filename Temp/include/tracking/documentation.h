/*

The tracking folder contains all functions relevent to the automous navigation and driving of the robot. 
This set of files contains the abstractions from the math to the robot design level.

The design philsophy revolves around the robot model classes that derive from the robot class. Inside a tracking thread the position and heading of the robot will be continously updated.
The main thread can then set desired targets and the robot instence will then output the correct speeds to each motor to get to the target.

In order of abstraction:
-robotmath.h - Contains all the abstract math functions including the Point and Vector Classes
-logger.h - Adds functionality releated to outputting the tracked path of the robot in a format readable to python programs and humans
-navigation.h - Tracks the position of the robot; Takes Position changes or sets as input and outputs a target vector and heading error
-robotModels.h - Takes the errors from navigation and employs PID to convert them to motor commands; Contains the robot model and derivive classes based on the design of the drive base

The least abstract classes should inherit and reference more abstract classes and additions should not be added to reverse the flow.


robotmath.h (Includes math.h and stdlib.h)
Written by: Carson Easterling
--------------------------------------------------------------------------


Constant of PI used to store and convert Radians
const double PI = 3.14159265


Absolute value functions - returns the input as positive
int abs(int x)
double abs(double x)

Sign function - Returns -1, 0, or 1 depending on the sign of the input
int sign(int x)
double sign(int x)

Floor function - Rounds the input down to the closest interger
int floorInt(double x)


Angle Conversions
double degToRad(double degrees) - Converts degrees to radians
double radToDeg(double radians) - Converts radians to degrees


Normalize Angle - Returns an input angle as an angle in given units bounded between [0, 2pi) or [0, 360)
double normalizeAngle(double theta, bool inRadians=true)


Smallest angle - Returns the smallest difference between 2 headings in radians; + if the target heading is counter clockwise of the current heading and - if clockwise
double shortestArcToTarget(double currentHeading, double targetHeading, bool inDeg=false)


Takes two headings and returns the shortest angle between the current heading and a secent line through the circle that passes through the target heading for example it will return the angle between the current and the target angle + 180 if that was the smaller angle
double shortestArcToLine(double currentHeading, double targetHeading, bool inDeg=false)


PID Algorithm
Returns an output (ex speed) that tries to minimize the error value (ex distance from target); Uses 3 tunable constants to get the output to decrease the error

Error: SetValue - CurrentValue; Larger error is farther than where it is trying to be
deltaT: Length of time since the last cycle
gains: The constants used to determine the output
previous: The results from the previous cycle

PIDOutput PID(double error, double deltaT, PIDGains gains, PIDOutput previous){
  double p = gains.p*error; //Proportinal - Larger error creates a larger output (Robot starts off fast and slows down as error decreases and it gets closer to target)
  double i = previous.reset + gains.i*error*deltaT; //Intergral - The longer error persits the output will increases and causes error to eventually approch zero
  double d = gains.d*(error - previous.lastError)/deltaT; //Derivitive - The larger the last change in error the more output will decrease to prevent overshooting of targt
  return {p+i+d, i, error}; //Output is P+I+D, the intergral is saved for the next cycle so it can increase if needed, error is saves for the next cycle for the derivitive can be calculated
}
For more reading: https://pidexplained.com/how-to-tune-a-pid-controller/


PIDGains - Struct that contains the coefficents used in the PID equation
PIDGains varName = {p, i, d};
varName.p - Contant mutiplied by the error - Larger error leds to larger output
varName.i - Containt mutiplied by the sum of the error - Longer time in error leds to a boost in output
varName.d - Constant multipled by the derivitive of the last cycle - Larger change in output leds to a decreased change in this output (To prevent overshoot of target value)


PID Output - Stores relevent infomation for the next PID cycle
PIDOutput varName = initPID(double initalError);
OR
PIDOutput varName = PID(...);
varName.output - Output of the PID cycle
varName.reset - Running sum of all errors
varName.lastError - The error used in the last cycle


Point, Vector, Beizer curve documentation goes here
































*/