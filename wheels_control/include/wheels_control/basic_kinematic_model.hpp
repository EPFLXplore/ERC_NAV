
#ifndef ROVER_BASIC_KINEMATIC_MODEL_HPP
#define ROVER_BASIC_KINEMATIC_MODEL_HPP



#include "definition.hpp"

enum ModeType {
    TRANSLATION_ONLY = 0,
    SET_ROTATION_POSITION = 1,
    ROTATION_ONLY = 2,
    SET_TRANSLATION_POSITION = 3,
    ROTATION_TRANSLATION = 4,
    CRABE = 10
};


class RoverBasicKinematicModel 
{
    private:
        int motion_mode;
        _Float64 wheels_angle_for_rotation;
        motors_obj current_motors_cmds;
        motors_obj current_motors_position;

        void translation(_Float64 norm_speed);

        void set_rotation_position(int wheels_angle);

        void set_translation_position();

        void rotation(_Float64 r_z, int wheels_angle);

        bool check_steering_position_for_translation() const;

        bool check_steering_position_for_rotation() const;

    public:
        RoverBasicKinematicModel();

        void init(motors_obj motors_position, _Float64 wheels_angle);

        motors_obj run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z);
};

#endif // ROVER_BASIC_KINEMATIC_MODEL_HPP
