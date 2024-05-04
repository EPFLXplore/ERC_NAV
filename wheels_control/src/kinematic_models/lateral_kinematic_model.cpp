#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "wheels_control/lateral_kinematic_model.hpp"





motors_obj RoverLateralKinematicModel::run(bool left, bool right)
{
    //RCLCPP_INFO(get_logger(), "Left%d", left);   
    //RCLCPP_INFO(get_logger(), "Right%d", right);                   
    RCLCPP_INFO(rclcpp::get_logger("SC_motor_cmds"),"Left%d", left);
    RCLCPP_INFO(rclcpp::get_logger("SC_motor_cmds"),"Right%d", right);



    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI);
    _Float64 conversion_speed = 3600; // for 1m.s
    float alpha = 1.57 / conversion_angle;
    float v_x = 0;
    if(left == right){
       v_x=0;
    }
    else {
        if (left){
            v_x = 3600/2;
        }
        else{
            v_x = -3600/2;  
        }
    }

    current_motors_cmds.drive[FRONT_LEFT] = v_x;
    current_motors_cmds.drive[FRONT_RIGHT] = v_x;
    current_motors_cmds.drive[BACK_RIGHT] = v_x;
    current_motors_cmds.drive[BACK_LEFT] = v_x;

    current_motors_cmds.steer[FRONT_LEFT] = alpha;
    current_motors_cmds.steer[FRONT_RIGHT] = alpha;
    current_motors_cmds.steer[BACK_RIGHT] = alpha;
    current_motors_cmds.steer[BACK_LEFT] = alpha;

    return current_motors_cmds;
}



