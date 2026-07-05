#ifndef FOUR_WHEEL_DRIVE_H
#define FOUR_WHEEL_DRIVE_H

#include <Arduino.h>

class FourWheelDrive {
    public:
        FourWheelDrive(int l_front_pin, int l_back_pin, int r_front_pin, int r_back_pin, int odometry_pin=-1);
        
        /* Path planning */
        /* --------------------------------------- */
        /* Grid Based */
        a_star_init();
        /* Sampling Based */
        rrt_init();
        rrt_star_init();


        /* Control*/
        /* --------------------------------------- */
        band_bang_init();
        pid_init();
        pure_pursuit_init();
    private:
        int l_front_pin;
        int l_back_pin;
        int r_front_pin;
        int r_back_pin;
        int odometry_pin;
};

#endif