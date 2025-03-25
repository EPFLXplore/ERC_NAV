/*
Last updated:       22/11/2024
Rewritting author:  Cyril Goffin
Description:        Computes the wheels velocity commands and the steering angle commands 
                    based on the gamepad inputs.
*/

#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

#include "wheels_control/normal_kinematic_model.hpp"

RoverNormalKinematicModel::RoverNormalKinematicModel() : motor_cmds(true),
                                                         en_rotation_quoi(false),
                                                         wheels_angle_for_rotation(0),
                                                         current_motors_cmds({{""}, {0, 0, 0, 0}, {0, 0, 0, 0}}),
                                                         current_motors_position({{0, 0, 0, 0}, {0, 0, 0, 0}})
{
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

    _Float64 conversion_speed = 3600; // 1800; // for 0.5m.s                       ????????????
    _Float64 conversion_angle = (pow(2, TOUR_RESOLUTION_BITS)) / (2 * M_PI); //    ????????????


    _Float64 max_linear_velocity = 0.7; // in m/s 
    _Float64 max_angular_velocity = 0.7; // in rad/s
    _Float64 max_ratation_radius = 1; // in m

    // limit the linear velocity
    if (std::abs(v_x) > max_linear_velocity)
    {
        v_x = (v_x / std::abs(v_x)) * max_linear_velocity;
    }

    // limit the angular velocity
    if (std::abs(omega_z) > max_angular_velocity)
    {
        omega_z = (omega_z / std::abs(omega_z)) * max_angular_velocity;
    }

    // 3 DIFFERENT CASES: ONLY ROTATION ON ITSELF, ONLY TRANSLATION, TRANSLATION AND ROTATION
    if (omega_z != 0)
    {
        r_ = v_x / omega_z;

        if (r_ != 0)
        {
            // TRANSLATION AND ROTATION (curve motion)
            current_motors_cmds.info = "translation and rotation";

            _Float64 sign_r = std::abs(r_) / r_;
            _Float64 velocity_sign = std::abs(v_x) / v_x; // equivalent to   _Float64 velocity_sign = v_x >= 0 ? 1 : -1;    ?? 

            if (std::abs(r_) < max_ratation_radius)
            {
                r_ = max_ratation_radius * sign_r;

                omega_z = std::abs(omega_z) / omega_z * v_x; // ?????????????????????
            }

            alpha_ext = atan2((LENGTH / 2), (std::abs(r_) + WIDTH / 2));
            alpha_int = atan2((LENGTH / 2), (std::abs(r_) - WIDTH / 2));

            _Float64 r_ext = std::sqrt((std::abs(r_) + WIDTH / 2) * (std::abs(r_) + WIDTH / 2) + (LENGTH / 2) * (LENGTH / 2));
            _Float64 r_int = std::sqrt((std::abs(r_) - WIDTH / 2) * (std::abs(r_) - WIDTH / 2) + (LENGTH / 2) * (LENGTH / 2));

            v_ext = std::abs(omega_z) * r_ext * velocity_sign;
            v_int = std::abs(omega_z) * r_int * velocity_sign;
        }
        else if (r_ == 0)
        {
            // ROTATION ON ITSELF
            current_motors_cmds.info = "self rotation";

            en_rotation_quoi = true;

            alpha_ext = wheels_angle_for_rotation / conversion_angle; // constant value for crab mode
            alpha_int = -alpha_ext;

            v_ext = (std::abs(omega_z));
            v_int = -(std::abs(omega_z));
        }
    }
    else
    {
        // ONLY TRANSLATION
        current_motors_cmds.info = "translation";

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
        wheels_current_commands.velocity_1 = v_ext * conversion_speed;

        wheels_current_commands.velocity_1 = v_ext * conversion_speed;
    }

    if (en_rotation_quoi)
    {
        if (v_x != 0)
        {
            if (check_steering_position_for_translation(motors_position))
            {
                en_rotation_quoi = false;
            }
            else
            {
                wheels_current_commands.angle_2 = 0;
                wheels_current_commands.angle_1 = 0;

                wheels_current_commands.velocity_2 = 0;
                wheels_current_commands.velocity_1 = 0;
                current_motors_cmds.info = "fail checking translation";
            }
        }
        else if (v_x == 0)
        {
            if (check_steering_position_for_rotation(motors_position))
            {

                std::cout << "rotation " << std::endl;
            }
            else
            {
                wheels_current_commands.velocity_2 = 0;
                wheels_current_commands.velocity_1 = 0;
                current_motors_cmds.info = "fail checking rotation";
            }
        }
    }

    rotation_translation(wheels_current_commands);

    return current_motors_cmds;
}

void RoverNormalKinematicModel::rotation_translation(wheels_normal_kinematic_cmds motors_commands)
{

    current_motors_cmds.drive[FRONT_LEFT] = motors_commands.velocity_1;
    current_motors_cmds.drive[FRONT_RIGHT] = motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_RIGHT] = motors_commands.velocity_2;
    current_motors_cmds.drive[BACK_LEFT] = motors_commands.velocity_1;

    current_motors_cmds.steer[FRONT_LEFT] = -motors_commands.angle_1;
    current_motors_cmds.steer[FRONT_RIGHT] = motors_commands.angle_2;
    current_motors_cmds.steer[BACK_RIGHT] = motors_commands.angle_2;
    current_motors_cmds.steer[BACK_LEFT] = -motors_commands.angle_1;
}

bool RoverNormalKinematicModel::check_steering_position_for_translation(motors_obj current_motors_position) const
{
    if (motor_cmds == true)
    {
        int wheel_positioned_for_translation = 0;

        for (auto motor = FRONT_LEFT; motor <= BACK_LEFT; motor++)
        {
            if (std::abs(current_motors_position.steer[motor]) < 50)
                wheel_positioned_for_translation++;
        }

        return (wheel_positioned_for_translation == NB_WHEELS);
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return true;
    }
}

bool RoverNormalKinematicModel::check_steering_position_for_rotation(motors_obj current_motors_position) const
{
    if (motor_cmds == true)
    {
        int wheel_positioned_for_rotation = 0;

        for (auto motor = 0; motor < NB_WHEELS; motor++)
        {
            if (std::abs(std::abs(current_motors_position.steer[motor]) - wheels_angle_for_rotation) < 50) // 1000
            {
                wheel_positioned_for_rotation++;
            }
        }

        return (wheel_positioned_for_rotation == NB_WHEELS);
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return true;
    }
}
