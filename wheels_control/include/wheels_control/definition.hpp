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

// NEW rover CAN Node IDs as of Sat 25th April 2026

#define FRONT_LEFT_DRIVE 1
#define FRONT_RIGHT_DRIVE 4
#define BACK_RIGHT_DRIVE 3
#define BACK_LEFT_DRIVE 2
#define FRONT_LEFT_STEER 5
#define FRONT_RIGHT_STEER 8
#define BACK_RIGHT_STEER 7
#define BACK_LEFT_STEER 6


enum class MotorKind { Drive, Steer };

struct MotorLayout {
    int can_id;
    int corner;
    MotorKind kind;
    const char* name;
};

inline constexpr MotorLayout MOTOR_LAYOUT[] = {
    {FRONT_LEFT_DRIVE,  FRONT_LEFT,  MotorKind::Drive, "FRONT_LEFT_DRIVE"},
    {BACK_LEFT_DRIVE,   BACK_LEFT,   MotorKind::Drive, "BACK_LEFT_DRIVE"},
    {BACK_RIGHT_DRIVE,  BACK_RIGHT,  MotorKind::Drive, "BACK_RIGHT_DRIVE"},
    {FRONT_RIGHT_DRIVE, FRONT_RIGHT, MotorKind::Drive, "FRONT_RIGHT_DRIVE"},
    {FRONT_LEFT_STEER,  FRONT_LEFT,  MotorKind::Steer, "FRONT_LEFT_STEER"},
    {BACK_LEFT_STEER,   BACK_LEFT,   MotorKind::Steer, "BACK_LEFT_STEER"},
    {BACK_RIGHT_STEER,  BACK_RIGHT,  MotorKind::Steer, "BACK_RIGHT_STEER"},
    {FRONT_RIGHT_STEER, FRONT_RIGHT, MotorKind::Steer, "FRONT_RIGHT_STEER"},
};

inline constexpr bool is_drive_motor(const MotorLayout& motor)
{
    return motor.kind == MotorKind::Drive;
}

inline constexpr bool is_steer_motor(const MotorLayout& motor)
{
    return motor.kind == MotorKind::Steer;
}

inline constexpr int motor_status_index(const MotorLayout& motor)
{
    return is_drive_motor(motor) ? motor.corner : motor.corner + NB_WHEELS;
}

inline constexpr int internal_can_index(const MotorLayout& motor)
{
    return motor.can_id - 1;
}

inline constexpr const MotorLayout* motor_layout_from_can_id(int can_id)
{
    for (const auto& motor : MOTOR_LAYOUT) {
        if (motor.can_id == can_id) {
            return &motor;
        }
    }
    return nullptr;
}

inline constexpr const MotorLayout* paired_motor_layout(const MotorLayout& motor)
{
    const MotorKind paired_kind = is_drive_motor(motor) ? MotorKind::Steer : MotorKind::Drive;
    for (const auto& candidate : MOTOR_LAYOUT) {
        if (candidate.corner == motor.corner && candidate.kind == paired_kind) {
            return &candidate;
        }
    }
    return nullptr;
}

#define TOUR_RESOLUTION_BITS 16
#define STEERING_RESOLUTION_BITS 14
#define TOUR_NB_INC 16384
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
#define JOYSTICK_THRESHOLD 0.005


#define RADIUS_MAX  1
#define RADIUS_MIN 0.5

#define NORMAL_KINEMATIC "normal"
#define NORMAL_KINEMATIC_SLOW "normal_slow"
#define LATERAL_KINEMATIC "lateral"

#define CURRENT_LIMIT 0

#define MIN_DESIRED_RADIUS 1
#define PI_IN_INCR 16384

// NEW rover dimensions as of Sat 25th April 2026
#define WIDTH 0.78
#define LENGTH 1.0

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
    AUTO = 3,
};



#endif
