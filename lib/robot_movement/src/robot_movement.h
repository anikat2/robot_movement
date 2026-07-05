#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <Arduino.h>

class RobotMovement {
    public:
        RobotMovement(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin, int odometry_pin=-1); //4 wheel drive constructor
        RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int end_effector_pin); //4 DOF arm constructor
        RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int roll_pin, int pitch_pin, int yaw_pin); //6 DOF arm constructor
    private:
};
#endif