#ifndef DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
#define DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H

#include <string>
#include <iostream>
#include <cmath>

struct Wheel
{
    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;
};

class Wheels_to_vel_and_dir
{
public:
    Wheel left_wheel_;
    Wheel right_wheel_;
    double wheel_separation_;
    double wheel_radius_;

    // Default constructor
    Wheels_to_vel_and_dir(){}
    

    void getVelocityAndDirection(float &speed, float &x, float &y)
    {   
        // Calculate wheel speeds (v_left and v_right)
        float v_left = left_wheel_.cmd * wheel_radius_;
        float v_right = right_wheel_.cmd * wheel_radius_;

        // Calculate forward velocity (v) and angular velocity (omega)
        float v = (v_left + v_right) / 2.0;
        float omega = (v_left - v_right) / wheel_separation_;

        // cmd between -5 and 5 when driving forward/ backward or between -1,3125 and 1,3125 when rotating on the spot
        // |v_max| = 2 * 5 * wheel_radius_ / 2 = 5 * wheel_radius_
        // |omega_max| = 2 * 1,3125 * wheel_radius_ / wheel_separation_
        float v_max = 5 * wheel_radius_;
        float omega_max = 2 * 1.3125 * wheel_radius_ / wheel_separation_;

        // calculate x and y and normalize them between 0 and 1
        y = v / v_max;              // forward movement
        x = omega / omega_max;      // rotation

        // Speed between 0 and 100%
        speed = sqrt(x * x + y * y) * 100.0;
    }
};

#endif // DIFFDRIVE_WHEELS_TO_VEL_AND_DIR_H
