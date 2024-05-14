#ifndef ROVER_S_TBY_KINEMATIC_MODEL_HPP
#define ROVER_S_TBY_KINEMATIC_MODEL_HPP



#include "definition.hpp"

class RoverStbyKinematicModel 
{
    private:
        motors_obj current_motors_cmds;
        motors_obj current_motors_position;
        int translation_sign = 0; // we will use this to either go to right or left


        void translation(_Float64 norm_speed);
        void rotation(float alpha);
        bool check_steering_position_for_rotation(motors_obj current_motors_position) const ;
        

    public:
        RoverStbyKinematicModel();

        void init(motors_obj motors_position, _Float64 wheels_angle);

        motors_obj run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 r_z);

        void set_sign_pos();
        void set_sign_neg();
};

#endif // ROVER_Stby_KINEMATIC_MODEL_HPP
