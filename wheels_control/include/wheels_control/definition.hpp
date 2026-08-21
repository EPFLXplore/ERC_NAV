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

// maxon limits
// #define MAX_DRIVE_ACCEL 900   // [rpm/S] // default :   900
// #define MAX_DRIVE_DECEL 3000  // [rpm/s] // default :  3000
// #define MAX_STEER_VEL 7   // [rpm]   // max 10
// #define MAX_STEER_ACCEL 20 // [rpm/s] // max 28


// Gamepad Definitions
#define GP_BUTTON_JOYSTICK_LEFT 7         // control crab mode in normal mode
#define GP_BUTTON_CROSS 0                 // Switch kinematics state
#define GP_BUTTON_ROUND 1                 // Switch mode rover
#define GP_AXIS_R2 5                      // NORMAL MODE: FORWARD | LATERAL: NOTHING
#define GP_AXIS_L2 2                      // NORMAL MODE: BACKWARD | LATERAL: NOTHING
#define GP_AXIS_JOYSTICK_LEFT_VERTICAL 1  // NORMAL MODE: Up/Down | LATERAL: MOVE FORWARD AND BACKWARD 
#define GP_AXIS_JOYSTICK_LEFT_HORIZONTAL 0  // 
#define JOYSTICK_THRESHOLD 0.005


#define NORMAL_KINEMATIC "normal"
#define NORMAL_KINEMATIC_SLOW "normal_slow"
#define LATERAL_KINEMATIC "lateral"


// NEW rover dimensions as of Wed. 1st July 2026
#define WIDTH 0.75
#define LENGTH 0.98
#define WHEEL_RADIUS 0.12 //quand ecrase un peu
#define MAX_LIN_VEL 2.4   //m/s at the wheel
#define MAX_STEER_VEL 7.0 //rpm
#define DIST_CENTER_WHEEL (std::sqrt((WIDTH/2)*(WIDTH/2) + (LENGTH/2)*(LENGTH/2)))
#define MAX_STEER_RATE (MAX_STEER_VEL * 2.0 * M_PI / 60.0)

// Unit conversions, shared by the kinematics, the servoing and the odometry so that a
// command and the measurement that comes back are expressed in the very same units.
inline constexpr double DRIVE_GEAR_RATIO = 53.0; // motor turns per wheel turn

// steering: encoder increments <-> [rad] of wheel steering
inline const double INCR_TO_RAD = 2.0 * M_PI / std::pow(2.0, STEERING_RESOLUTION_BITS);
inline const double RAD_TO_INCR = 1.0 / INCR_TO_RAD;

// drive: motor rpm (before the gearbox, as commanded and as reported) <-> [m/s] at the ground
inline const double MS_TO_DRIVE_RPM = (60.0 * DRIVE_GEAR_RATIO) / (2.0 * M_PI * WHEEL_RADIUS);
inline const double DRIVE_RPM_TO_MS = 1.0 / MS_TO_DRIVE_RPM;

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
