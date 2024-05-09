#ifndef ROVER_LATERAL_KINEMATIC_MODEL_HPP
#define ROVER_LATERAL_KINEMATIC_MODEL_HPP



#include "definition.hpp"




class RoverLateralKinematicModel 
{
    private:
        
        motors_obj current_motors_cmds;
        motors_obj current_motors_position;


    public:
        //RoverLateralKinematicModel();

        //void init(motors_obj motors_position, _Float64 wheels_angle);

        motors_obj run(bool left, bool right);
};

#endif // ROVER_LATERAL_KINEMATIC_MODEL_HPP
