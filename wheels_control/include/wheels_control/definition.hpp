#pragma once

#ifndef DEFINITION_HPP
#define DEFINITION_HPP

#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <string>
#include <cstdint>


#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3

#define NB_WHEELS 4 
#define NB_MOTORS 8 

#define FRONT_LEFT_DRIVE 1
#define FRONT_RIGHT_DRIVE 2
#define BACK_RIGHT_DRIVE 3
#define BACK_LEFT_DRIVE 4
#define FRONT_LEFT_STEER 5
#define FRONT_RIGHT_STEER 6
#define BACK_RIGHT_STEER 7
#define BACK_LEFT_STEER 8

#define TOUR_RESOLUTION_BITS 16
#define TOUR_NB_INC 65536
#define WHEELS_INC_PRECISION 300


#define SPEED_ROVER_MIN 0
#define SPEED_ROVER_MAX 1000
#define SPEED_ROVER_ROT_MIN 0
#define SPEED_ROVER_ROT_MAX 500
#define SPEED_CMD_MIN 0
#define SPEED_CMD_MAX 1 // 2¹5
#define ANGLE_MAX 30 //2900000/8 // = 45
#define ANGLE_MIN 0

// Gamepad Definitions
#define GP_BUTTON_JOYSTICK_LEFT 7         // control crab mode in normal mode
#define GP_BUTTON_CROSS 0                 // Switch kinematics state
#define GP_BUTTON_ROUND 1                 // Switch mode rover
#define GP_AXIS_R2 5                      // NORMAL MODE: FORWARD | LATERAL: NOTHING
#define GP_AXIS_L2 2                      // NORMAL MODE: BACKWARD | LATERAL: NOTHING
#define GP_AXIS_JOYSTICK_LEFT_VERTICAL 1  // NORMAL MODE: Up/Down | LATERAL: MOVE FORWARD AND BACKWARD 
#define GP_AXIS_JOYSTICK_LEFT_HORIZONTAL 0  // 
#define JOYSTICK_THRESHOLD 0.00001


#define RADIUS_MAX  1
#define RADIUS_MIN 0.5

#define NORMAL_KINEMATIC "normal"
#define NORMAL_KINEMATIC_SLOW "normal_slow"
#define LATERAL_KINEMATIC "lateral"

#define CURRENT_LIMIT 0

#define MIN_DESIRED_RADIUS 1
#define PI_IN_INCR 16384


#define WIDTH 0.736
#define LENGTH 0.833

// #define ROTATION_TRANSLATION 0
// #define CRABE 1
// #define ROTATION_ONLY 2
// #define TRANSLATION_ONLY 3

struct motors_obj {
    std::string   info;
    _Float64 steer[NB_WHEELS];
    _Float64 drive[NB_WHEELS];
};

struct wheels_normal_kinematic_cmds {
    _Float64 velocity_1;
    _Float64 velocity_2;
    _Float64 angle_1;
    _Float64 angle_2;
};

enum ROVER_MODE {
    OFF = 0,
    ACKERMANN = 1,
    OMNI_DIRECTIONAL = 2,
    AUTO = 3
};



#endif