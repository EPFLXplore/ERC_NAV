#include <cmath>
#include <iostream>  

#include "wheels_control/diagonal_kinematic_model.hpp"


RoverDiagonalKinematicModel::RoverDiagonalKinematicModel() : 
    motion_mode(TRANSLATION_ONLY), current_motors_cmds({{""},{0, 0, 0, 0}, {0, 0, 0, 0}}),
    current_motors_position({{0, 0, 0, 0}, {0, 0, 0, 0}}) {
}

void RoverDiagonalKinematicModel::init(motors_obj motors_position)
{
    current_motors_position = motors_position;
}

motors_obj RoverDiagonalKinematicModel::run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z)
{
    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI);
    float alpha = 0.785 * conversion_angle; // pi/4
    current_motors_position = motors_position;
    int translation_sign = 0;


    
    _Float64 norm_speed = std::sqrt(v_x * v_x + v_y * v_y);

    int speed = norm_speed * (SPEED_ROVER_MAX - SPEED_ROVER_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);
    int speed_rot = r_z * (SPEED_ROVER_ROT_MAX - SPEED_ROVER_ROT_MIN) / (SPEED_CMD_MAX - SPEED_CMD_MIN);

    if(r_z > 0.15 && (v_x < 0.1) ) // faut changer ca par si laxe de la joystick pointe en haut ou en bas
    {

        if (check_steering_position_for_translation()) // faut avoir les roue à 0 deg
        {
            motion_mode = TRANSLATION_ONLY;
        }
        else
        {
           motion_mode = SET_TRANSLATION_POSITION;
        }
        translation_sign = v_x/abs(v_x) * (-1); // pourquoi cest comme ca? nest ce pas juste -1 ou +1?
    }
    else if( "ici on check si laxe est pointe a gauche")
    {
        if (check_steering_position_for_rotation(alpha))
        {
            motion_mode = ROTATION_LEFT;
        }
        else
        {
            motion_mode = SET_ROTATION_POSITION;    
        }
    }
    else
    {
        // seul autre option cest que cest orienté a droite
        if (check_steering_position_for_rotation(alpha))
        {
            motion_mode = ROTATION_RIGHT;
        }
        else
        {
            motion_mode = SET_ROTATION_POSITION;    
        }
    }
    
    
    
    switch (motion_mode) 
    {
        case TRANSLATION_ONLY:
            translation(translation_sign*speed);
            current_motors_cmds.info = "translation only";
            break;

        case ROTATION_LEFT:
            int angle_sign = -1;
            rotation(angle_sign*alpha);
            if(check_steering_position_for_drive(current_motors_position))
            {
                translation(translation_sign*speed);
            }
            else
            {
                current_motors_cmds.drive[FRONT_LEFT] = 0;
                current_motors_cmds.drive[FRONT_RIGHT] = 0;
                current_motors_cmds.drive[BACK_RIGHT] = 0;
                current_motors_cmds.drive[BACK_LEFT] = 0;
            }
            current_motors_cmds.info = "rotation left";
            break;

        case ROTATION_RIGHT:
            rotation(alpha);
            if(check_steering_position_for_drive(current_motors_position))
            {
                translation(translation_sign*speed);
            }
            else
            {
                current_motors_cmds.drive[FRONT_LEFT] = 0;
                current_motors_cmds.drive[FRONT_RIGHT] = 0;
                current_motors_cmds.drive[BACK_RIGHT] = 0;
                current_motors_cmds.drive[BACK_LEFT] = 0;
            }
            current_motors_cmds.info = "rotation left";
            break;

        case SET_TRANSLATION_POSITION:
            set_translation_position();
            current_motors_cmds.info = "set translation";
            break;

        case SET_ROTATION_POSITION:
            set_rotation_position(alpha);
            current_motors_cmds.info = "set rotation";
            break;

        default:
            break;
    }

    return current_motors_cmds;

}

bool RoverDiagonalKinematicModel::check_steering_position_for_translation() const 
{
    int wheel_positioned_for_translation = 0;

    for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++) {
        if (current_motors_position.steer[motor] < WHEELS_INC_PRECISION)
            wheel_positioned_for_translation++;
    }

    return wheel_positioned_for_translation == NB_WHEELS;
}

bool RoverDiagonalKinematicModel::check_steering_position_for_rotation(float alpha) const 
{
    int wheel_positioned_for_rotation = 0;

    for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++)
    {
    if (std::abs(current_motors_position.steer[motor]) >  alpha - 200)
        wheel_positioned_for_rotation++;
    }

    return wheel_positioned_for_rotation == NB_WHEELS;
}

bool RoverDiagonalKinematicModel::check_steering_position_for_drive(motors_obj current_motors_position) const 
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

void RoverDiagonalKinematicModel::set_translation_position() 
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

void RoverDiagonalKinematicModel::translation(_Float64 norm_speed) 
{
    /* Commands motors for just a basic translation */
    current_motors_cmds.drive[FRONT_LEFT] = norm_speed;
    current_motors_cmds.drive[FRONT_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_RIGHT] = norm_speed;
    current_motors_cmds.drive[BACK_LEFT] = norm_speed;
}

void RoverDiagonalKinematicModel::rotation(float alpha) 
{
    current_motors_cmds.steer[FRONT_LEFT] = alpha;
    current_motors_cmds.steer[FRONT_RIGHT] = -alpha;
    current_motors_cmds.steer[BACK_RIGHT] = alpha;
    current_motors_cmds.steer[BACK_LEFT] = -alpha;
}

// je connais deja langle dorientation que je veux faire si cest pas uniquement de la translation que je veux faire.
// faut faire un if sur les axes de la joystick pour savoir chaque etat.
// jai deux mode de rotation : -pi/4 et +pi/4

//pi/4 en increment:  (pi*0.25)* (pow(2, TOUR_RESOLUTION_BITS))) / (2 * M_PI); 