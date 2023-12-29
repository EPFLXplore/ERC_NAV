#include <cmath>
#include <iostream>

#include "normal_kinematic_model.hpp"


RoverNormalKinematicModel::RoverNormalKinematicModel():
    en_rotation_quoi(false),
    wheels_angle_for_rotation(0),
    current_motors_cmds({{""},{0, 0, 0, 0}, {0, 0, 0, 0}}),
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
    _Float64 v_ext = 0.0;
    _Float64 v_int = 0.0;

    _Float64 alpha_ext = 0.0;
    _Float64 alpha_int = 0.0;

    _Float64 conversion_speed = 3600; // for 1m.s
    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI);

    
    if (std::abs(v_x) > 0.5)
    {
        v_x = (v_x / std::abs(v_x)) * 0.5;
    }

    // if (std::abs(omega_z) > 0.5)
    // {
    //     omega_z = (omega_z / std::abs(omega_z)) * 0.5;
    // }
    

    if (omega_z != 0)
    {
        r_ = v_x / omega_z;
        std::cout << "TRANSLATION AND ROTATION "  << std::endl;

        std::cout << "r_1 " << r_ << std::endl;

        
        if (r_ != 0)
        {
            _Float64 sign_r = std::abs(r_)/r_;

            if(std::abs(r_) < 1)
            {
                r_ = 1 * sign_r;

                // if (std::abs(v_x) == 0.5)
                // {
                //     std::cout << "OMEGA LIMITS " << r_ << std::endl;
                //     omega_z = v_x;

                // }
                omega_z = std::abs(omega_z) / omega_z *  v_x;

            }

            std::cout << "v_x " << v_x << std::endl;

            std::cout << "omega_z " << omega_z << std::endl;

            std::cout << "r_2 " << r_ << std::endl;
            std::cout << "desired radius " << r_ << std::endl;


            alpha_ext = atan2((LENGTH/2),(std::abs(r_) + WIDTH/2)); // atan2 between -pi and pi
            alpha_int = atan2((LENGTH/2),(std::abs(r_)- WIDTH/2));

            _Float64 velocity_sign = v_x >= 0 ? 1 : -1;

            _Float64 r_ext = std::sqrt((std::abs(r_) + WIDTH/2) * (std::abs(r_) + WIDTH/2) + (LENGTH/2) * (LENGTH/2));
            _Float64 r_int = std::sqrt((std::abs(r_) - WIDTH/2) * (std::abs(r_) - WIDTH/2) + (LENGTH/2) * (LENGTH/2));

            std::cout << "r_ext " << r_ext << std::endl;
            std::cout << "r_int " << r_int << std::endl;

            v_ext = std::abs(omega_z) * r_ext * velocity_sign;
            v_int = std::abs(omega_z) * r_int * velocity_sign;
            current_motors_cmds.info = "translation and rotation";

        }
        else if (r_ == 0)
        {
            _Float64 omega_z_local = 0.0;

            current_motors_cmds.info = "self rotation";
            en_rotation_quoi = true;
            std::cout << "wheels_angle_for_rotation " << wheels_angle_for_rotation << std::endl;
            alpha_ext = wheels_angle_for_rotation / conversion_angle;
            alpha_int = - wheels_angle_for_rotation / conversion_angle;

            

            if (std::abs(omega_z) > 0.5)
            {
                omega_z_local = (omega_z / std::abs(omega_z)) * 0.5;
            }
            else
            {
                omega_z_local = omega_z;
            }

            v_ext = (std::abs(omega_z_local)) ;
            v_int = - (std::abs(omega_z_local)) ;

        }


        // if (std::abs(v_ext) > 0.5)
        // {
        //     v_ext = (v_ext / std::abs(v_ext)) * 0.5;
        // }

        

        std::cout << "alpha_ext " << alpha_ext << std::endl;
        std::cout << "alpha_int " << alpha_int << std::endl;

        std::cout << "v_ext " << v_ext << std::endl;
        std::cout << "v_int " << v_int << std::endl;
    }
    //else if (omega_z == 0)
    else
    {
        current_motors_cmds.info = "translation";
        std::cout << "TRANSLATION "  << std::endl;
        // if (std::abs(v_x) > 0.5)
        // {
        //     v_x = (v_x / std::abs(v_x)) * 0.5;
        // }

        v_ext = v_x;
        v_int = v_x;
        alpha_ext = 0;
        alpha_int = 0;
    }

    wheels_normal_kinematic_cmds wheels_current_commands;

   
    if (omega_z >= 0)
    {

        wheels_current_commands.angle_1 = alpha_int * conversion_angle;
        wheels_current_commands.angle_2 = alpha_ext * conversion_angle;

        wheels_current_commands.velocity_1 = v_int * conversion_speed;
        wheels_current_commands.velocity_2 = v_ext * conversion_speed;

    }
    else
    {

        wheels_current_commands.angle_2 = -alpha_int * conversion_angle;
        wheels_current_commands.angle_1 = -alpha_ext * conversion_angle;

        wheels_current_commands.velocity_2 = v_int * conversion_speed;
        wheels_current_commands.velocity_1 = v_ext * conversion_speed;  

    }
    

    // if ( (std::abs(wheels_current_commands.angle_2) > PI_IN_INCR))
    // {
    //     wheels_current_commands.velocity_2 = -wheels_current_commands.velocity_2;
    //     _Float64 sign =  omega_z >= 0 ? 1 : -1;
    //     wheels_current_commands.angle_2 = sign * (int(std::abs(wheels_current_commands.angle_2)) % PI_IN_INCR);

    // }
    // else if ((std::abs(wheels_current_commands.angle_1) > PI_IN_INCR))
    // {
    //     wheels_current_commands.velocity_1 = -wheels_current_commands.velocity_1;
    //     _Float64 sign = omega_z >= 0 ? 1 : -1;
    //     wheels_current_commands.angle_1 = sign *(int(std::abs(wheels_current_commands.angle_1)) % PI_IN_INCR);
    // }
    std::cout << "en_rotation_quoi " << en_rotation_quoi << std::endl;
    if (en_rotation_quoi)
    {
        if(v_x != 0)
        {
            if(check_steering_position_for_translation(motors_position))
            {
                std::cout << "translation " << std::endl;
                en_rotation_quoi = false;
            }
            else
            {
                std::cout << "translation NOOO" << std::endl;
                wheels_current_commands.angle_2 = 0;
                wheels_current_commands.angle_1 = 0;

                wheels_current_commands.velocity_2 = 0;
                wheels_current_commands.velocity_1 = 0; 
                current_motors_cmds.info = "fail checking translation";
            }
        }
        else if (v_x == 0)
        {
            if(check_steering_position_for_rotation(motors_position))
            {
                
                std::cout << "rotation " << std::endl;

            }
            else
            {
                wheels_current_commands.velocity_2 = 0;
                wheels_current_commands.velocity_1 = 0; 
                std::cout << "rotation NOO " << std::endl;
                current_motors_cmds.info = "fail checking rotation";
            }
                

        }

    }

    rotation_translation(wheels_current_commands);

    
    return current_motors_cmds;
}



void RoverNormalKinematicModel::rotation_translation(wheels_normal_kinematic_cmds motors_commands)
{

    current_motors_cmds.drive[FRONT_LEFT] =  motors_commands.velocity_1;
    current_motors_cmds.drive[FRONT_RIGHT] =  motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_RIGHT] =  motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_LEFT] =  motors_commands.velocity_1;

    current_motors_cmds.steer[FRONT_LEFT] = -motors_commands.angle_1;
    current_motors_cmds.steer[FRONT_RIGHT] = motors_commands.angle_2;
    current_motors_cmds.steer[BACK_RIGHT] = motors_commands.angle_2 ;
    current_motors_cmds.steer[BACK_LEFT] =  -motors_commands.angle_1 ;

}



bool RoverNormalKinematicModel::check_steering_position_for_translation(motors_obj current_motors_position) const 
{
    int wheel_positioned_for_translation = 0;

    for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++)
    {
        if (std::abs(current_motors_position.steer[motor]) < 50)
            wheel_positioned_for_translation++;
    }

    std::cout << "T "<< wheel_positioned_for_translation << std::endl;
            

    return (wheel_positioned_for_translation == NB_WHEELS);
}



bool RoverNormalKinematicModel::check_steering_position_for_rotation(motors_obj current_motors_position) const 
{
    int wheel_positioned_for_rotation = 0;

    for (auto motor = 0; motor < NB_WHEELS; motor++) 
    {
        if (std::abs(std::abs(current_motors_position.steer[motor]) - wheels_angle_for_rotation) < 50)  //1000
        {
            wheel_positioned_for_rotation++;
        }
    }

    std::cout << "R "<< wheel_positioned_for_rotation << std::endl;

    return (wheel_positioned_for_rotation == NB_WHEELS);
}