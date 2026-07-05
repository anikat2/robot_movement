#ifndef 4_PLUS_DOF_ARM_H
#define 4_PLUS_DOF_ARM_H

#include <Arduino.h>

class 4PlusDOFArm {
    public:
        4PlusDOFArm(int waist_pin, int shoulder_pin, int elbow_pin, int end_effector_pin);
        4PlusDOFArm(int waist_pin, int shoulder_pin, int elbow_pin, int roll_pin, int pitch_pin, int yaw_pin);      
        inverse_kineatics_init();
        forward_kinematics_init();
    private:
}

#endif