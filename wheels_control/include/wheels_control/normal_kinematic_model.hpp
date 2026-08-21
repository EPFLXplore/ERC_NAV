#ifndef ROVER_NORMAL_KINEMATIC_MODEL_HPP
#define ROVER_NORMAL_KINEMATIC_MODEL_HPP

#include "definition.hpp"

// struct wheels_normal_kinematic_cmds
// {
//     _Float64 velocity_1;
//     _Float64 velocity_2;
//     _Float64 angle_1;
//     _Float64 angle_2;
// };

class RoverNormalKinematicModel
{
private:
    bool en_rotation_quoi;
    _Float64 wheels_angle_for_rotation;
    motors_obj current_motors_cmds;
    motors_obj current_motors_position;

    // last commanded steering angle of each side [rad], used to pick the slip-ring
    // equivalent (angle +- k*pi) that is closest to where the steering already is
    double previous_angle_left;
    double previous_angle_right;

    void rotation_translation(wheels_normal_kinematic_cmds motors_kinematic_commands);

    bool check_steering_position_for_translation(motors_obj current_motors_position) const;

    bool check_steering_position_for_rotation(motors_obj current_motors_position) const;

public:
    bool motor_cmds;
    RoverNormalKinematicModel();

    void init(motors_obj motors_position, _Float64 wheels_angle);

    // v_x [m/s], v_y [m/s] (unused), r_z [rad/s] ROS convention, speed_rover [m/s] wheel speed cap
    motors_obj run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z, _Float64 speed_rover, bool crab_mode_active);
};

#endif // ROVER_NORMAL_KINEMATIC_MODEL_HPP
