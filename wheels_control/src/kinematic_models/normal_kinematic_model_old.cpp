#include <cmath>
#include <iostream>  

#include "normal_kinematic_model.hpp"


RoverNormalKinematicModel::RoverNormalKinematicModel():
    current_motors_cmds({{0, 0, 0, 0}, {0, 0, 0, 0}}),
    current_motors_position({{0, 0, 0, 0}, {0, 0, 0, 0}}) {
}



void RoverNormalKinematicModel::init(motors_obj motors_position, _Float64 wheels_angle)
{
    wheels_angle_for_rotation = wheels_angle;
    current_motors_position = motors_position;
}



motors_obj RoverNormalKinematicModel::run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 omega_z)
{

    _Float64 r_ = 0;
    if (omega_z != 0)
    {
        r_ = v_x / omega_z;
    }

    _Float64 conversion_speed = 3600;
    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI);


    _Float64 width = 0.736;
    _Float64 length = 0.833;

    _Float64 r_l = std::sqrt((r_ + width/2) * (r_ + width/2) + (length/2) * (length/2));
    _Float64 r_r = std::sqrt((r_ - width/2) * (r_ - width/2) + (length/2) * (length/2));

    _Float64 v_l = omega_z * r_l;
    _Float64 v_r = omega_z * r_r;

    _Float64 alpha_l = atan((length/2)/(r_ + width/2));
    _Float64 alpha_r = atan((length/2)/(r_ - width/2));

    if (omega_z == 0)
    {
        v_l = v_x;
        v_r = v_x;
        alpha_l = 0;
        alpha_r = 0;
    }

    wheels_normal_kinematic_cmds wheels_current_commands;

    wheels_current_commands.angle_1 = 0;//alpha_r * conversion_angle;
    wheels_current_commands.angle_2 = 0;//alpha_l * conversion_angle;

    wheels_current_commands.velocity_1 = 300; //v_r * conversion_speed;
    wheels_current_commands.velocity_2 = 300; //v_l * conversion_speed;

    rotation_translation(wheels_current_commands);

    return current_motors_cmds;
}

void RoverNormalKinematicModel::rotation_translation(wheels_normal_kinematic_cmds motors_commands)
{

    current_motors_cmds.drive[FRONT_LEFT] =  motors_commands.velocity_1;
    current_motors_cmds.drive[FRONT_RIGHT] =  motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_RIGHT] =  motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_LEFT] =  motors_commands.velocity_1;

    current_motors_cmds.steer[FRONT_LEFT] = motors_commands.angle_1;
    current_motors_cmds.steer[FRONT_RIGHT] = motors_commands.angle_2;
    current_motors_cmds.steer[BACK_RIGHT] = motors_commands.angle_2;
    current_motors_cmds.steer[BACK_LEFT] =  motors_commands.angle_1;

}
