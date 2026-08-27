/*
Last updated:       21/08/2026
Rewritting author:  Arno Laurie, Paul Bourgois
Description:        Computes the wheels velocity commands and the steering angle commands
                    from a body twist (v_x [m/s], omega_z [rad/s], ROS convention).

Kinematics:

    For a rigid body, the ground velocity of the wheel at body offset (x_i, y_i) is

        v_ix = v_x - omega_z * y_i
        v_iy =       omega_z * x_i

    steering angle  delta_i = atan2(v_iy, v_ix)
    wheel speed     s_i     = hypot(v_ix, v_iy)

    This is exact for every radius (including 0) and every direction of travel, so
    no Ackermann inner/outer case split and no singularity is needed.

    With x_i = +-LENGTH/2 and y_i = +-WIDTH/2 the front and rear wheel of one side
    only differ by the sign of delta, which the front/rear mirroring in
    rotation_translation() already accounts for. Only the left pair (index 1) and
    the right pair (index 2) are therefore computed here.
*/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include "wheels_control/utility.hpp"
#include "custom_msg/msg/motorcmds.hpp"
#include "custom_msg/msg/wheelstatus.hpp"
#include "wheels_control/normal_kinematic_model.hpp"
#include "wheels_control/definition.hpp"
#include "wheels_control/motors.hpp"


//use the ros logger instead of cout from iostream
#include "rclcpp/rclcpp.hpp"
void log_info_acker(const std::string &msg){
    RCLCPP_INFO(rclcpp::get_logger("acker kinematics"), "%s", msg.c_str());
}

namespace
{

constexpr double EPS = 1e-6;



struct wheel_state
{
    double angle; // [rad], steering angle of the front wheel of that side, ROS convention
    double speed; // [m/s], signed along the wheel heading
};

/*
 * brief :  ground velocity of one wheel for a body twist, expressed as a steering
 *          angle and a signed rolling speed.
 * param :  x_off, y_off    wheel position in the body frame [m]
 * param :  previous_angle  last commanded angle for that steering axis [rad]
 *
 */
wheel_state wheel_from_twist(
    double v_x,
    double omega_z,
    double x_off,
    double y_off,
    double previous_angle)
{
    const double v_wx = v_x - omega_z * y_off;
    const double v_wy = omega_z * x_off;

    if (std::hypot(v_wx, v_wy) < EPS)
        return {0.0, 0.0};
        // return {previous_angle, 0.0};

    const double raw_angle = std::atan2(v_wy, v_wx);

    const double delta =
        std::remainder(raw_angle - previous_angle, M_PI);

    const double angle = previous_angle + delta;

    /*
     * Determine whether the wheel must drive forward or backward
     * along the selected steering axis.
     */
    const double speed =
        v_wx * std::cos(angle) +
        v_wy * std::sin(angle);

    return {angle, speed};
}
} // namespace

RoverNormalKinematicModel::RoverNormalKinematicModel() : en_rotation_quoi(false),
                                                         wheels_angle_for_rotation(0),
                                                         current_motors_cmds({"", {0, 0, 0, 0}, {0, 0, 0, 0}}),
                                                         current_motors_position({"", {0, 0, 0, 0}, {0, 0, 0, 0}}),
                                                         previous_angle_left(0.0),
                                                         previous_angle_right(0.0),
                                                         motor_cmds(true)
{
}

void RoverNormalKinematicModel::init(motors_obj motors_position, _Float64 wheels_angle)
{
    wheels_angle_for_rotation = wheels_angle;
    current_motors_position = motors_position;
    previous_angle_left = 0.0;
    previous_angle_right = 0.0;
}

/*
 * v_x      [m/s]   forward body velocity
 * v_y      [m/s]   unused, an ackermann rover cannot translate sideways (see the lateral model)
 * omega_z  [rad/s] yaw rate, ROS convention (positive = turning left)
 * speed_rover [m/s] wheel speed cap requested by the CS
 */
motors_obj RoverNormalKinematicModel::run(motors_obj motors_position, _Float64 v_x, _Float64 v_y, _Float64 omega_z, _Float64 speed_rover, bool crab_mode_active)
{
    (void)v_y;
    current_motors_position = motors_position;

    const double max_wheel_speed = std::min(static_cast<double>(speed_rover),
                                            static_cast<double>(MAX_LIN_VEL));

    // Turning in place is only allowed when the operator asked for it (crab button), otherwise
    // a steering input alone must not make the rover pivot. In AUTO the caller sets the flag.
    if (std::abs(v_x) < EPS && !crab_mode_active)
    {
        omega_z = 0.0;
    }

    if (std::abs(v_x) < EPS && std::abs(omega_z) < EPS)
        current_motors_cmds.info = "idle";
    else if (std::abs(omega_z) < EPS)
        current_motors_cmds.info = "translation";
    else if (std::abs(v_x) < EPS)
        current_motors_cmds.info = "self rotation";
    else
        current_motors_cmds.info = "translation and rotation";

    wheel_state left  = wheel_from_twist(v_x, omega_z,  LENGTH / 2.0,  WIDTH / 2.0, previous_angle_left);
    wheel_state right = wheel_from_twist(v_x, omega_z,  LENGTH / 2.0, -WIDTH / 2.0, previous_angle_right);

    // Scale both sides by the same factor: this keeps the instantaneous centre of rotation
    // (and therefore the path and the steering angles) untouched, the rover just goes slower.
    const double fastest_wheel = std::max(std::abs(left.speed), std::abs(right.speed));
    if (fastest_wheel > max_wheel_speed && fastest_wheel > EPS)
    {
        const double scale = max_wheel_speed / fastest_wheel;
        left.speed *= scale;
        right.speed *= scale;
    }

    previous_angle_left = left.angle;
    previous_angle_right = right.angle;

    // The hardware convention of rotation_translation() is the negated ROS steering angle.
    wheels_normal_kinematic_cmds wheels_current_commands;
    wheels_current_commands.angle_1 = -left.angle * RAD_TO_INCR;
    wheels_current_commands.angle_2 = -right.angle * RAD_TO_INCR;
    wheels_current_commands.velocity_1 = left.speed * MS_TO_DRIVE_RPM;
    wheels_current_commands.velocity_2 = right.speed * MS_TO_DRIVE_RPM;

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
