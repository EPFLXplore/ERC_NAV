
#include <cmath>
#include <iostream>  

#include "basic_kinematic_model.hpp"



RoverBasicKinematicModel::RoverBasicKinematicModel() : 
    motion_mode(TRANSLATION_ONLY), wheels_angle_for_rotation(0.0),
    current_motors_cmds({{""},{0, 0, 0, 0}, {0, 0, 0, 0}}),
    current_motors_position({{0, 0, 0, 0}, {0, 0, 0, 0}}) {
}



void RoverBasicKinematicModel::init(motors_obj motors_position, _Float64 wheels_angle)
{
    wheels_angle_for_rotation = wheels_angle;
    current_motors_position = motors_position;
}



motors_obj RoverBasicKinematicModel::run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z)
{
    /* Rotation or translation */
    current_motors_position = motors_position;

    _Float64 norm_speed = std::sqrt(v_x * v_x + v_y * v_y);

    int speed = norm_speed * (SPEED_ROVER_MAX - SPEED_ROVER_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);
    int speed_rot = r_z * (SPEED_ROVER_ROT_MAX - SPEED_ROVER_ROT_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);

    int translation_sign = 0;
    

    // Manage the motion mode
    //if ((r_z > 0.1 ||  r_z <-0.1 ) && (v_x < 0.1))  // r_z !=0
    if(std::abs(r_z) > 0.15 && (v_x < 0.1) )
    {
        if (check_steering_position_for_rotation())
        {
            motion_mode = ROTATION_ONLY;
            std::cout << "Hello, World!" << std::endl;
        }
        else
        {
            motion_mode = SET_ROTATION_POSITION;    
            //set_rotation_position(wheels_angle_for_rotation);

        }
            

    } 
    else //if (v_x != 0 && (r_z < 0.1 ||  r_z <-0.1 ) ) 
    {
        if (check_steering_position_for_translation())
            motion_mode = TRANSLATION_ONLY;
        else
            motion_mode = SET_TRANSLATION_POSITION;

        translation_sign = v_x/abs(v_x) * (-1);
    }

    // Get the appropriate commands for the motors
    switch (motion_mode) 
    {
        case TRANSLATION_ONLY:
            translation(translation_sign * speed);
            current_motors_cmds.info = "translation only";
            break;

        case ROTATION_ONLY:
            rotation(speed_rot, wheels_angle_for_rotation);
            current_motors_cmds.info = "rotation only";
            break;

        case SET_TRANSLATION_POSITION:
            set_translation_position();
            current_motors_cmds.info = "set translation";
            break;

        case SET_ROTATION_POSITION:
            set_rotation_position(wheels_angle_for_rotation);
            current_motors_cmds.info = "set rotation";
            break;

        default:
            break;
    }

    return current_motors_cmds;
}




void RoverBasicKinematicModel::translation(_Float64 norm_speed) 
{
    /* Commands motors for just a basic translation */
    current_motors_cmds.drive[FRONT_LEFT] = norm_speed;
    current_motors_cmds.drive[FRONT_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_LEFT] = norm_speed;

    current_motors_cmds.steer[FRONT_LEFT] = 0;
    current_motors_cmds.steer[FRONT_RIGHT] = 0; // steering motor mounted in reverse
    current_motors_cmds.steer[BACK_RIGHT] = 0;
    current_motors_cmds.steer[BACK_LEFT] = 0;
}


void RoverBasicKinematicModel::set_rotation_position(int wheels_angle) 
{
    /* Put the steering motors in position to run the rotation */
    current_motors_cmds.drive[FRONT_LEFT] = 0;
    current_motors_cmds.drive[FRONT_RIGHT] = 0;
    current_motors_cmds.drive[BACK_RIGHT] = 0;
    current_motors_cmds.drive[BACK_LEFT] = 0;

    current_motors_cmds.steer[FRONT_LEFT] = wheels_angle;
    current_motors_cmds.steer[FRONT_RIGHT] = wheels_angle;
    current_motors_cmds.steer[BACK_RIGHT] = wheels_angle;
    current_motors_cmds.steer[BACK_LEFT] = wheels_angle;
}


void RoverBasicKinematicModel::set_translation_position() 
{
    /* Put the steering motors in position to run a basic translation */
    current_motors_cmds.drive[FRONT_LEFT] = 0;
    current_motors_cmds.drive[FRONT_RIGHT] = 0;
    current_motors_cmds.drive[BACK_RIGHT] = 0;
    current_motors_cmds.drive[BACK_LEFT] = 0;

    current_motors_cmds.steer[FRONT_LEFT] = 0;
    current_motors_cmds.steer[FRONT_RIGHT] = 0;
    current_motors_cmds.steer[BACK_RIGHT] = 0;
    current_motors_cmds.steer[BACK_LEFT] = 0;
}


void RoverBasicKinematicModel::rotation(_Float64 r_z, int wheels_angle) 
{

    /* Commands motors for rover rotation */
    current_motors_cmds.drive[FRONT_LEFT] = r_z;
    current_motors_cmds.drive[FRONT_RIGHT] = -r_z;
    current_motors_cmds.drive[BACK_RIGHT] = -r_z;
    current_motors_cmds.drive[BACK_LEFT] = r_z;

    current_motors_cmds.steer[FRONT_LEFT] = wheels_angle;
    current_motors_cmds.steer[FRONT_RIGHT] = wheels_angle;
    current_motors_cmds.steer[BACK_RIGHT] = wheels_angle;
    current_motors_cmds.steer[BACK_LEFT] = wheels_angle;
}


bool RoverBasicKinematicModel::check_steering_position_for_translation() const 
{
    int wheel_positioned_for_translation = 0;

    for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++) {
        if (current_motors_position.steer[motor] < WHEELS_INC_PRECISION)
            wheel_positioned_for_translation++;
    }

    return wheel_positioned_for_translation == NB_WHEELS;
}


bool RoverBasicKinematicModel::check_steering_position_for_rotation() const 
{
    int wheel_positioned_for_rotation = 0;

    for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++)
    {
    if (std::abs(current_motors_position.steer[motor]) >  wheels_angle_for_rotation - 200)
        wheel_positioned_for_rotation++;
    }

    return wheel_positioned_for_rotation == NB_WHEELS;
}

