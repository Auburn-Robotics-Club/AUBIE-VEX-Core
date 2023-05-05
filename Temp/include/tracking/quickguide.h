/*******************************************************************

Quick guide for fast auto adjustments:


The robot's starting position on the field is considered as (0, 0) and the front of the robot is typically considered 90 degrees heading
All angles are positive in the counter-clockwise direction

The following functions are implied to have a "robot."" in front of each of them to control the robot
For example "robot.setAbsTarget(10, 100.2);" is how you would call a function



Managing robot speed
maxV = X (Where X is between 0-100 default is 100) - Maximum tangential velocity
maxW = X (Where X is between 0-100 default is 100) - Maximum rotational velocity
cruiseVelocity = X (Where X is between 0-100) - Constant velcoity during curves

stopMotors() - Will not allow the automatic updates to the motors from the robot control system (Used if motors need to be controled via another sensor ex camera or using time)
activatePID() - Will allow the robot to control its own speed to reach a target
activateSpeedTarget(double speed, double angularSpeed) - Will reach a target at a constant specified speed (Inputs should each be between 0-100)
activateCV(Vector cv, double w) - Will ignore any current target and will travel in a vector (dX is horizontal (only for XDrive) and dY is front back) or will rotate (w is short for angular velocity)


Front forward mode: Hvae the front of the robot turn in the direction of your target and drive only forward
Has to be used for tank drives but could also could be used for XDrives

Could also be reversed using
setDir(bool fwd=true)

To set a target based on the absolute grid (The grid the robot starts on and doesn't depend on any point past the robots starting position)

Goes to a specific location
setAbsTarget(Point p)
-Point p = Point(x, y)

Changes target location based on last target location
setAbsTarget(double deltaX, double deltaY)
setAbsTarget(Vector v)
-Vector V = Vector(double magnitude, double theta, bool inDegrees)
-Vector V = Vector(double delta_x, double delta_y)
-Vector V = Vector(Point start, Point end)

Headless mode: For X drives that can move and rotate indpendently
setAbsTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false)
setAbsTarget(Vector v, double targetHead, bool inDeg=false)
setAbsTarget(Point p, double targetHead, bool inDeg=false)


To change a target realitve to the robot's last target position and heading (So that the last heading is considered 90 degrees and the last point is defined as (0, 0) on this grid)

Front Forward:
setRelTarget(double deltaX, double deltaY)
setRelTarget(Vector v)

Headless:
setRelTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false)
setRelTarget(Vector v, double targetHead, bool inDeg=false)

Turning:
Turns to an absolute heading based on starting heading
void turnTo(double head, bool inDeg=false)

Turns a realitive amount based on current heading
void turn(double deltaHeading, bool inDeg=false)

Hold:

executeMove() will hold until the robot gets to the target desitnation

Example X-Drive Program:
std::cout << "HI" << std::endl;
robot.activatePID();
robot.setAbsTarget(0, 36);
executeMove();
robot.setAbsTarget(36, 0);
executeMove();
robot.setAbsTarget(-36, -36, 90, true);
executeMove();
std::cout << "END" << std::endl;


*/