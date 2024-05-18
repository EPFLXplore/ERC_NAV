
#ifndef ROVER_DIAGONAL_KINEMATIC_MODEL_HPP
#define ROVER_DIAGONAL_KINEMATIC_MODEL_HPP



#include "definition.hpp"

enum ModeType {
    TRANSLATION_ONLY = 0,
    SET_ROTATION_POSITION = 1,
    SET_TRANSLATION_POSITION = 2,
    ROTATION_LEFT = 3,
    ROTATION_RIGHT = 4,
    CRABE = 10
};

class RoverDiagonalKinematicModel 
{
    private:
        int motion_mode;
        motors_obj current_motors_cmds;
        motors_obj current_motors_position;

        void translation(_Float64 norm_speed);

        void set_rotation_position(int wheels_angle);

        void set_translation_position();

        void rotation(float alpha);

        bool check_steering_position_for_translation() const;

        bool check_steering_position_for_rotation(float alpha) const;

        bool check_steering_position_for_drive(motors_obj current_motors_position) const;

    public:
        RoverDiagonalKinematicModel();

        void init(motors_obj motors_position);

        motors_obj run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z);
};

#endif // ROVER_DIAGONAL_KINEMATIC_MODEL_HPP
