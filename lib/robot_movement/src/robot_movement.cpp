#include "robot_movement.h"

RobotMovement::RobotMovement(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin,
                             int odometry_pin)
    : _type(ROBOT_FOUR_WHEEL),
      _drive(l_front_pin, l_back_pin, r_front_pin, r_back_pin, odometry_pin),
      _arm_dof(0) {
    for (uint8_t i = 0; i < 6; i++) {
        _arm_pins[i] = -1;
    }
}

RobotMovement::RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int end_effector_pin)
    : _type(ROBOT_ARM_4DOF), _drive(-1, -1, -1, -1, -1), _arm_dof(4) {
    _arm_pins[0] = waist_pin;
    _arm_pins[1] = shoulder_pin;
    _arm_pins[2] = elbow_pin;
    _arm_pins[3] = end_effector_pin;
    _arm_pins[4] = -1;
    _arm_pins[5] = -1;
}

RobotMovement::RobotMovement(int waist_pin, int shoulder_pin, int elbow_pin, int roll_pin,
                             int pitch_pin, int yaw_pin)
    : _type(ROBOT_ARM_6DOF), _drive(-1, -1, -1, -1, -1), _arm_dof(6) {
    _arm_pins[0] = waist_pin;
    _arm_pins[1] = shoulder_pin;
    _arm_pins[2] = elbow_pin;
    _arm_pins[3] = roll_pin;
    _arm_pins[4] = pitch_pin;
    _arm_pins[5] = yaw_pin;
}

RobotType RobotMovement::type() const {
    return _type;
}

FourWheelDrive &RobotMovement::drive() {
    return _drive;
}
