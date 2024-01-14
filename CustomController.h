#pragma once
#include "v5.h"
#include "v5_vcs.h"
#include "robotmath/robotmath.h"

using namespace vex;

/**
 * Wrapper class for vex::controller with custom functions.
 * Original vex::controller functions can be accessed via CustomController::internal.
*/
class CustomController {
    public:
        controller internal;

    private:
        class Axis {
            private:
                controller* internal;
                int id;

                double position() {
                   switch(id) {
                        case 1:
                            return internal->Axis1.position();

                        case 2:
                            return internal->Axis2.position();
                        
                        case 3:
                            return internal->Axis3.position();

                        case 4:
                            return internal->Axis4.position();
                   }
                }

            public:
                Axis(controller* c, int i) {
                    internal = c;
                    id = i;
                }

                /**
                 * Gets the position of the joystick axis on a scale from -100 to 100.
                 * A deadzone of 3 is implemented.
                 * 
                 * @return Returns an integer that represents the position of the joystick axis.
                */
                double get() {
                    float deadband = 3;
                    double value = position();
                    if(abs(value) < deadband) {
                        return 0;
                    }
                    return value;
                }
        };

        class Button {
            private:
                controller* internal;
                int id;
                bool wasPressed;
                bool wasReleased;

                bool pressing() {
                    switch(id) {
                        case 1:
                            return internal->ButtonA.pressing();
                        
                        case 2:
                            return internal->ButtonB.pressing();
                        
                        case 3:
                            return internal->ButtonX.pressing();
                        
                        case 4:
                            return internal->ButtonY.pressing();
                        
                        case 5:
                            return internal->ButtonUp.pressing();
                        
                        case 6:
                            return internal->ButtonDown.pressing();
                        
                        case 7:
                            return internal->ButtonLeft.pressing();
                        
                        case 8:
                            return internal->ButtonRight.pressing();
                        
                        case 9:
                            return internal->ButtonL1.pressing();
                        
                        case 10:
                            return internal->ButtonL2.pressing();
                        
                        case 11:
                            return internal->ButtonR1.pressing();
                        
                        case 12:
                            return internal->ButtonR2.pressing();
                    }
                }
            
            public:
                Button(controller* c, int i) {
                    internal = c;
                    id = i;
                }

                /**
                 * Gets the status of a button.
                 * (Identical to vex::controller::button::pressing())
                 * 
                 * @return Returns a Boolean value based on the pressed states of the button. If the button is pressed it will return true.
                */
                bool get() {
                    return pressing();
                }

                /**
                 * Get a single button press.
                 * 
                 * @return A boolean for whether a single button press has occurred.
                */
                bool pressed() {
                    if(pressing()) {
                        if(wasPressed) {
                            return false;
                        }
                        wasPressed = true;
                        return true;
                    }
                    wasPressed = false;
                    return false;
                }

                /**
                 * Get a single button release.
                 * 
                 * @return A boolean for whether a single button release has occurred.
                */
                bool released() {
                    if(!pressing()) {
                        if(wasReleased) {
                            return false;
                        }
                        wasReleased = true;
                        return true;
                    }
                    wasReleased = false;
                    return false;
                }
        };

    public:
        /**
         * Horizontal axis for the right stick.
        */
        Axis Axis1;
        /**
         * Vertical axis for the right stick.
        */
        Axis Axis2;
        /**
         * Vertical axis for the left stick.
        */
        Axis Axis3;
        /**
         * Horizontal axis for the left stick
        */
        Axis Axis4;

        Button
        ButtonA, ButtonB, ButtonX, ButtonY,
        ButtonUp, ButtonDown, ButtonLeft, ButtonRight,
        ButtonL1, ButtonL2, ButtonR1, ButtonR2;

        CustomController(controllerType id) :
        internal(id),
        Axis1(&internal, 1),
        Axis2(&internal, 2),
        Axis3(&internal, 3),
        Axis4(&internal, 4),
        ButtonA(&internal, 1),
        ButtonB(&internal, 2),
        ButtonX(&internal, 3),
        ButtonY(&internal, 4),
        ButtonUp(&internal, 5),
        ButtonDown(&internal, 6),
        ButtonLeft(&internal, 7),
        ButtonRight(&internal, 8),
        ButtonL1(&internal, 9),
        ButtonL2(&internal, 10),
        ButtonR1(&internal, 11),
        ButtonR2(&internal, 12) {}

        /**
         * Rumbles the controller by a pattern defined by the parameter.
         * Dots equal short; dashes equal long and space equals pause.
         * (Identical to vex::controller::rumble())
         * 
         * @param str A string that consists of dots and dashes that represent the rumble pattern.
        */
        void rumble(const char* str) {
            internal.rumble(str);
        }
};