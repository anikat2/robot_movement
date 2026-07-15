#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <Arduino.h>
#include "4_wheel_drive/4_wheel_drive.h"

enum RobotType : uint8_t {
    ROBOT_NONE = 0,
    ROBOT_FOUR_WHEEL = 1,
    ROBOT_ARM_4DOF = 2,
    ROBOT_ARM_6DOF = 3
};

class RobotMovement {
public:
    RobotMovement(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin,
                  int odometry_pin = -1);
    RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int end_effector_pin);
    RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int roll_pin, int pitch_pin,
                  int yaw_pin);

    RobotType type() const;
    FourWheelDrive &drive();

private:
    RobotType _type;
    FourWheelDrive _drive;
    int _arm_pins[6];
    uint8_t _arm_dof;
};

#endif
