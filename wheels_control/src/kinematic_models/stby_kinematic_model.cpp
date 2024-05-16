#include <cmath>
#include <iostream>  

#include "wheels_control/stby_kinematic_model.hpp"




RoverStbyKinematicModel::RoverStbyKinematicModel() : 
    current_motors_cmds({{""},{0, 0, 0, 0}, {0, 0, 0, 0}}),
    current_motors_position({{0, 0, 0, 0}, {0, 0, 0, 0}}) {
}



void RoverStbyKinematicModel::init(motors_obj motors_position, _Float64 wheels_angle)
{
    current_motors_position = motors_position;
}



motors_obj RoverStbyKinematicModel::run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z)
{
    /* Rotation or translation */
    current_motors_position = motors_position;
    _Float64 norm_speed = std::sqrt(v_x * v_x + v_y * v_y);
    //int speed = norm_speed * (SPEED_ROVER_MAX - SPEED_ROVER_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);
    int speed = v_x * (SPEED_ROVER_MAX - SPEED_ROVER_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);

    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI);
    _Float64 conversion_speed = 3600; // for 1m.s
    float alpha = 1.57 * conversion_angle;


    // Get the appropriate commands for the motors
    rotation(alpha);
    if(check_steering_position_for_rotation(current_motors_position)){
        translation(speed);
    }else{

        current_motors_cmds.drive[FRONT_LEFT] = 0;
        current_motors_cmds.drive[FRONT_RIGHT] = 0;
        current_motors_cmds.drive[BACK_RIGHT] = 0;
        current_motors_cmds.drive[BACK_LEFT] = 0;
     }
    
    current_motors_cmds.info = "translation only";
    return current_motors_cmds;
}






void RoverStbyKinematicModel::translation(_Float64 norm_speed) 
{
    
    /* Commands motors for just a basic translation */
    current_motors_cmds.drive[FRONT_LEFT] = norm_speed;
    current_motors_cmds.drive[FRONT_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_LEFT] = norm_speed;
}


void RoverStbyKinematicModel::rotation(float alpha) 
{
    current_motors_cmds.steer[FRONT_LEFT] = alpha;
    current_motors_cmds.steer[FRONT_RIGHT] = -alpha;
    current_motors_cmds.steer[BACK_RIGHT] = alpha;
    current_motors_cmds.steer[BACK_LEFT] = -alpha;
}


void RoverStbyKinematicModel::set_sign_neg()
{
    translation_sign = -1;
}

void RoverStbyKinematicModel::set_sign_pos()
{
    translation_sign = 1;
}

bool RoverStbyKinematicModel::check_steering_position_for_rotation(motors_obj current_motors_position) const 
{
    int wheel_positioned_for_rotation = 0;

    for (auto motor = 0; motor < NB_WHEELS; motor++) 
    {
        if (std::abs(std::abs(current_motors_position.steer[motor]) - 90) < 50)  // 50 cest un petit interval et 90 cest langle quon veut 
        {
            wheel_positioned_for_rotation++;
        }
    }

    std::cout << "R "<< wheel_positioned_for_rotation << std::endl;

    return (wheel_positioned_for_rotation == NB_WHEELS);
}