#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "wheels_control/lateral_kinematic_model.hpp"





motors_obj RoverLateralKinematicModel::run(double v_x, double r_z)
{
                  
    //RCLCPP_INFO(rclcpp::get_logger("omnidrive"),"omnikinematics vx %f", v_x);
    //RCLCPP_INFO(rclcpp::get_logger("omnidrive"),"kinematics angle%f", r_z);

    _Float64 conversion_angle = (pow(2, STEERING_RESOLUTION_BITS)) / (2 * M_PI);
    _Float64 conversion_speed = 3600; // for 1m.s

    const double max_speed = 0.8; //m/s
    double omni_speed = max_speed * v_x *conversion_speed; 
    float alpha = (-1.0) * (r_z) * conversion_angle;

    current_motors_cmds.steer[FRONT_LEFT] = alpha;
    current_motors_cmds.steer[FRONT_RIGHT] = -alpha;
    current_motors_cmds.steer[BACK_RIGHT] = alpha;
    current_motors_cmds.steer[BACK_LEFT] = -alpha;
    //RCLCPP_INFO(rclcpp::get_logger("omnidrive"),"omnikin speed vx %f", omni_speed);

    current_motors_cmds.drive[FRONT_LEFT] = omni_speed;
    current_motors_cmds.drive[FRONT_RIGHT] = omni_speed;
    current_motors_cmds.drive[BACK_RIGHT] = omni_speed;
    current_motors_cmds.drive[BACK_LEFT] = omni_speed;
    
    return current_motors_cmds;
}



