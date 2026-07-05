#include "robot_movement.h"

RobotMovement::RobotMovement(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin, int odometry_pin=-1){
    if (odometry_pin != -1) {
        pins = {l_front_pin, l_back_pin, r_front_pin, r_back_pin, odometry_pin};
    } else {
    pins = {l_front_pin, l_back_pin, r_front_pin, r_back_pin};
    }
}
