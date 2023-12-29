#ifndef ROVER_SLOW_NORMAL_KINEMATIC_MODEL_HPP
#define ROVER_SLOW_NORMAL_KINEMATIC_MODEL_HPP



#include "definition.hpp"

// struct wheels_normal_kinematic_cmds
// {
//     _Float64 velocity_1;
//     _Float64 velocity_2;
//     _Float64 angle_1;
//     _Float64 angle_2;
// };


class RoverSlowNormalKinematicModel
{
    private:
        bool en_rotation_quoi;
        _Float64 wheels_angle_for_rotation;
        motors_obj current_motors_cmds;
        motors_obj current_motors_position;


        void rotation_translation(wheels_normal_kinematic_cmds motors_kinematic_commands);

     
        wheels_normal_kinematic_cmds kinematics_motion_translation_rotation(_Float64 velocity,
                     _Float64 desired_radius);


        bool check_steering_position_for_translation(motors_obj current_motors_position) const;

        bool check_steering_position_for_rotation(motors_obj current_motors_position) const ;



    public:
        RoverSlowNormalKinematicModel();


        void init(motors_obj motors_position, _Float64 wheels_angle);

        motors_obj run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z);
};

#endif // ROVER_SLOW_NORMAL_KINEMATIC_MODEL_HPP
