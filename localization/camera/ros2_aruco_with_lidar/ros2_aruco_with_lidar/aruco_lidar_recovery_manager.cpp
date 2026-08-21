/*
 * Supervises the map-guided ArUco/LiDAR crop and switches it to a
 * camera-guided recovery mode only when several independent observations
 * indicate that map->odom no longer projects landmarks where the camera sees
 * them.  Returning to map guidance requires LiDAR cube detections, a
 * solver-approved rover pose, and sustained camera/map agreement.
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class ArucoLidarRecoveryManager : public rclcpp::Node
{
public:
    ArucoLidarRecoveryManager()
        : Node("aruco_lidar_recovery_manager"), tf_buffer_(get_clock())
    {
        auto_recovery_enabled_ =
            declare_parameter<bool>("auto_recovery_enabled", true);
        const bool initial_camera_mode =
            declare_parameter<bool>("use_camera_aruco_position", false);
        camera_observation_ttl_sec_ =
            declare_parameter<double>("camera_observation_ttl_sec", 0.8);
        lidar_success_ttl_sec_ =
            declare_parameter<double>("lidar_success_ttl_sec", 0.8);
        pose_solution_ttl_sec_ =
            declare_parameter<double>("pose_solution_ttl_sec", 1.0);
        enter_disagreement_m_ =
            declare_parameter<double>("enter_disagreement_m", 0.8);
        exit_disagreement_m_ =
            declare_parameter<double>("exit_disagreement_m", 0.35);
        displacement_consistency_m_ =
            declare_parameter<double>("displacement_consistency_m", 0.5);
        enter_hold_sec_ = declare_parameter<double>("enter_hold_sec", 0.6);
        exit_hold_sec_ = declare_parameter<double>("exit_hold_sec", 2.0);
        min_trigger_tags_ = declare_parameter<int>("min_trigger_tags", 2);
        min_recovery_lidar_tags_ =
            declare_parameter<int>("min_recovery_lidar_tags", 2);
        min_observable_range_m_ =
            declare_parameter<double>("min_observable_range_m", 0.5);
        max_observable_range_m_ =
            declare_parameter<double>("max_observable_range_m", 15.0);
        forbidden_angle_min_deg_ =
            declare_parameter<double>("forbidden_angle_min_deg", 110.0);
        forbidden_angle_max_deg_ =
            declare_parameter<double>("forbidden_angle_max_deg", 160.0);

        validate_parameters();

        state_ = initial_camera_mode ? State::CAMERA_RECOVERY : State::MAP_GUIDED;
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        camera_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", sensor_qos,
            std::bind(&ArucoLidarRecoveryManager::camera_callback, this,
                      std::placeholders::_1));
        lidar_sub_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/cube_markers", sensor_qos,
            std::bind(&ArucoLidarRecoveryManager::lidar_callback, this,
                      std::placeholders::_1));
        pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/aruco_rover_pos", sensor_qos,
            std::bind(&ArucoLidarRecoveryManager::pose_callback, this,
                      std::placeholders::_1));

        mode_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/perception/use_camera_aruco_position", mode_qos);
        state_pub_ = create_publisher<std_msgs::msg::String>(
            "/perception/aruco_lidar_recovery_state", mode_qos);

        parameter_callback_ = add_on_set_parameters_callback(
            std::bind(&ArucoLidarRecoveryManager::on_parameters, this,
                      std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ArucoLidarRecoveryManager::evaluate, this));

        publish_state("startup");
    }

private:
    enum class State { MAP_GUIDED, CAMERA_RECOVERY, REACQUIRE_MAP };

    struct TagObservation
    {
        int64_t receive_ns{0};
        double range_m{0.0};
        double displacement_x_m{0.0};
        double displacement_y_m{0.0};
        double disagreement_m{0.0};
        bool eligible{false};
        bool have_disagreement{false};
    };

    static const char *state_name(State state)
    {
        switch (state) {
            case State::MAP_GUIDED: return "MAP_GUIDED";
            case State::CAMERA_RECOVERY: return "CAMERA_RECOVERY";
            case State::REACQUIRE_MAP: return "REACQUIRE_MAP";
        }
        return "UNKNOWN";
    }

    bool camera_mode() const
    {
        return state_ != State::MAP_GUIDED;
    }

    void validate_parameters()
    {
        if (!(camera_observation_ttl_sec_ > 0.0) ||
            !(lidar_success_ttl_sec_ > 0.0) ||
            !(pose_solution_ttl_sec_ > 0.0) ||
            !(enter_disagreement_m_ > exit_disagreement_m_) ||
            !(exit_disagreement_m_ > 0.0) ||
            !(displacement_consistency_m_ > 0.0) ||
            !(enter_hold_sec_ > 0.0) || !(exit_hold_sec_ > 0.0) ||
            min_trigger_tags_ < 2 || min_recovery_lidar_tags_ < 2 ||
            !(max_observable_range_m_ > min_observable_range_m_) ||
            !(min_observable_range_m_ >= 0.0)) {
            throw std::runtime_error("invalid ArUco/LiDAR recovery parameters");
        }
    }

    bool transform_point(
        const geometry_msgs::msg::Point &point,
        const std::string &source_frame,
        const std::string &target_frame,
        const builtin_interfaces::msg::Time &stamp,
        geometry_msgs::msg::Point &result)
    {
        if (source_frame.empty() || target_frame.empty()) return false;

        geometry_msgs::msg::PointStamped input;
        input.header.frame_id = source_frame;
        input.header.stamp = stamp;
        input.point = point;

        try {
            geometry_msgs::msg::TransformStamped transform;
            const bool zero_stamp = stamp.sec == 0 && stamp.nanosec == 0u;
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        target_frame, source_frame, tf2::TimePointZero,
                        tf2::durationFromSec(0.05))) {
                    return false;
                }
                transform = tf_buffer_.lookupTransform(
                    target_frame, source_frame, tf2::TimePointZero);
            } else {
                const rclcpp::Time time(stamp, get_clock()->get_clock_type());
                if (!tf_buffer_.canTransform(
                        target_frame, source_frame, time,
                        rclcpp::Duration::from_seconds(0.05))) {
                    return false;
                }
                transform = tf_buffer_.lookupTransform(
                    target_frame, source_frame, time,
                    rclcpp::Duration::from_seconds(0.05));
            }

            geometry_msgs::msg::PointStamped output;
            tf2::doTransform(input, output, transform);
            result = output.point;
            return std::isfinite(result.x) && std::isfinite(result.y);
        } catch (const tf2::TransformException &) {
            return false;
        }
    }

    void camera_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        const size_t count = std::min({
            msg->marker_ids.size(), msg->poses.size(),
            msg->landmark_map_pos_x.size(), msg->landmark_map_pos_y.size(),
            msg->ar_angles_list.size()});
        const int64_t receive_ns = now().nanoseconds();
        std::unordered_map<int64_t, TagObservation> observations;

        for (size_t index = 0; index < count; ++index) {
            const int64_t id = msg->marker_ids[index];
            if (observations.find(id) != observations.end()) continue;

            geometry_msgs::msg::Point camera_in_base;
            if (!transform_point(
                    msg->poses[index].position, msg->header.frame_id, "base_link",
                    msg->header.stamp, camera_in_base)) {
                continue;
            }

            TagObservation observation;
            observation.receive_ns = receive_ns;
            observation.range_m = std::hypot(camera_in_base.x, camera_in_base.y);
            const double angle_deg = msg->ar_angles_list[index];
            const bool forbidden =
                angle_deg > forbidden_angle_min_deg_ &&
                angle_deg < forbidden_angle_max_deg_;
            observation.eligible =
                std::isfinite(observation.range_m) &&
                observation.range_m >= min_observable_range_m_ &&
                observation.range_m <= max_observable_range_m_ && !forbidden;

            const double map_x = msg->landmark_map_pos_x[index];
            const double map_y = msg->landmark_map_pos_y[index];
            if (observation.eligible && std::isfinite(map_x) && std::isfinite(map_y) &&
                std::fabs(map_x) < 90000.0 && std::fabs(map_y) < 90000.0) {
                geometry_msgs::msg::Point map_landmark;
                map_landmark.x = map_x;
                map_landmark.y = map_y;
                geometry_msgs::msg::Point map_in_base;
                if (transform_point(
                        map_landmark, "map", "base_link", msg->header.stamp,
                        map_in_base)) {
                    observation.displacement_x_m = camera_in_base.x - map_in_base.x;
                    observation.displacement_y_m = camera_in_base.y - map_in_base.y;
                    observation.disagreement_m = std::hypot(
                        observation.displacement_x_m,
                        observation.displacement_y_m);
                    observation.have_disagreement =
                        std::isfinite(observation.disagreement_m);
                }
            }
            observations.emplace(id, observation);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        camera_observations_ = std::move(observations);
    }

    void lidar_callback(
        const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        const int64_t receive_ns = now().nanoseconds();
        std::unordered_set<int64_t> unique_ids;
        for (const int64_t id : msg->marker_ids) unique_ids.insert(id);

        std::lock_guard<std::mutex> lock(mutex_);
        last_lidar_batch_ns_ = receive_ns;
        last_lidar_batch_size_ = unique_ids.size();
        for (const int64_t id : unique_ids) {
            last_lidar_success_ns_[id] = receive_ns;
        }
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (msg->header.frame_id != "map") return;
        const auto &position = msg->pose.pose.position;
        if (!std::isfinite(position.x) || !std::isfinite(position.y)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        last_pose_solution_ns_ = now().nanoseconds();
    }

    size_t largest_consistent_failure_cluster(
        const std::vector<TagObservation> &failures) const
    {
        size_t largest = 0;
        for (const auto &reference : failures) {
            size_t count = 0;
            for (const auto &candidate : failures) {
                if (std::hypot(
                        candidate.displacement_x_m - reference.displacement_x_m,
                        candidate.displacement_y_m - reference.displacement_y_m) <=
                    displacement_consistency_m_) {
                    ++count;
                }
            }
            largest = std::max(largest, count);
        }
        return largest;
    }

    void transition(State next, const std::string &reason, int64_t now_ns)
    {
        if (state_ == next) return;
        const State previous = state_;
        state_ = next;
        enter_evidence_start_ns_ = 0;
        exit_evidence_start_ns_ = 0;
        if (next == State::CAMERA_RECOVERY) recovery_enter_ns_ = now_ns;
        RCLCPP_WARN(get_logger(), "ArUco/LiDAR mode %s -> %s: %s",
            state_name(previous), state_name(next), reason.c_str());
        publish_state(reason);
    }

    void publish_state(const std::string &reason)
    {
        std_msgs::msg::Bool mode;
        mode.data = camera_mode();
        mode_pub_->publish(mode);

        std_msgs::msg::String state;
        state.data = std::string(state_name(state_)) + ": " + reason;
        state_pub_->publish(state);
    }

    void evaluate()
    {
        const int64_t now_ns = now().nanoseconds();
        const int64_t camera_ttl_ns =
            static_cast<int64_t>(camera_observation_ttl_sec_ * 1e9);
        const int64_t lidar_ttl_ns =
            static_cast<int64_t>(lidar_success_ttl_sec_ * 1e9);
        const int64_t pose_ttl_ns =
            static_cast<int64_t>(pose_solution_ttl_sec_ * 1e9);

        std::lock_guard<std::mutex> lock(mutex_);
        if (!auto_recovery_enabled_) return;

        std::vector<TagObservation> failures;
        size_t agreement_count = 0;
        size_t eligible_count = 0;
        for (const auto &[id, observation] : camera_observations_) {
            if (!observation.eligible || !observation.have_disagreement ||
                now_ns - observation.receive_ns > camera_ttl_ns) {
                continue;
            }
            ++eligible_count;
            const auto success = last_lidar_success_ns_.find(id);
            const bool lidar_recent =
                success != last_lidar_success_ns_.end() &&
                now_ns - success->second <= lidar_ttl_ns;
            if (!lidar_recent &&
                observation.disagreement_m >= enter_disagreement_m_) {
                failures.push_back(observation);
            }
            if (observation.disagreement_m <= exit_disagreement_m_) {
                ++agreement_count;
            }
        }

        const size_t failure_cluster = largest_consistent_failure_cluster(failures);
        const bool failure_evidence =
            failure_cluster >= static_cast<size_t>(min_trigger_tags_);
        const bool recovery_evidence =
            last_lidar_batch_size_ >= static_cast<size_t>(min_recovery_lidar_tags_) &&
            now_ns - last_lidar_batch_ns_ <= lidar_ttl_ns &&
            now_ns - last_pose_solution_ns_ <= pose_ttl_ns &&
            last_lidar_batch_ns_ > recovery_enter_ns_ &&
            last_pose_solution_ns_ > recovery_enter_ns_;
        const bool map_agreement =
            agreement_count >= static_cast<size_t>(min_trigger_tags_);

        if (state_ == State::MAP_GUIDED) {
            if (failure_evidence) {
                if (enter_evidence_start_ns_ == 0) enter_evidence_start_ns_ = now_ns;
                if (now_ns - enter_evidence_start_ns_ >=
                    static_cast<int64_t>(enter_hold_sec_ * 1e9)) {
                    std::ostringstream reason;
                    reason << failure_cluster
                           << " consistent camera/map disagreements with no LiDAR cube";
                    transition(State::CAMERA_RECOVERY, reason.str(), now_ns);
                }
            } else {
                enter_evidence_start_ns_ = 0;
            }
        } else if (state_ == State::CAMERA_RECOVERY) {
            if (recovery_evidence) {
                transition(State::REACQUIRE_MAP,
                    "camera-guided LiDAR and pose solver are producing valid results",
                    now_ns);
            }
        } else {
            if (failure_evidence) {
                transition(State::CAMERA_RECOVERY,
                    "camera/map disagreement returned during map reacquisition", now_ns);
            } else if (recovery_evidence && map_agreement) {
                if (exit_evidence_start_ns_ == 0) exit_evidence_start_ns_ = now_ns;
                if (now_ns - exit_evidence_start_ns_ >=
                    static_cast<int64_t>(exit_hold_sec_ * 1e9)) {
                    transition(State::MAP_GUIDED,
                        "LiDAR pose and camera/map agreement remained stable", now_ns);
                }
            } else {
                exit_evidence_start_ns_ = 0;
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "recovery=%s eligible=%zu failures=%zu consistent=%zu agreement=%zu "
            "lidar_batch=%zu pose_recent=%s",
            state_name(state_), eligible_count, failures.size(), failure_cluster,
            agreement_count, last_lidar_batch_size_,
            (now_ns - last_pose_solution_ns_ <= pose_ttl_ns) ? "yes" : "no");
    }

    rcl_interfaces::msg::SetParametersResult on_parameters(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "auto_recovery_enabled") {
                auto_recovery_enabled_ = parameter.as_bool();
            } else if (parameter.get_name() == "use_camera_aruco_position") {
                state_ = parameter.as_bool()
                    ? State::CAMERA_RECOVERY : State::MAP_GUIDED;
                recovery_enter_ns_ = now().nanoseconds();
                enter_evidence_start_ns_ = 0;
                exit_evidence_start_ns_ = 0;
                publish_state("manual parameter update");
            }
        }
        return result;
    }

    bool auto_recovery_enabled_{true};
    double camera_observation_ttl_sec_{0.8};
    double lidar_success_ttl_sec_{0.8};
    double pose_solution_ttl_sec_{1.0};
    double enter_disagreement_m_{0.8};
    double exit_disagreement_m_{0.35};
    double displacement_consistency_m_{0.5};
    double enter_hold_sec_{0.6};
    double exit_hold_sec_{2.0};
    int min_trigger_tags_{2};
    int min_recovery_lidar_tags_{2};
    double min_observable_range_m_{0.5};
    double max_observable_range_m_{15.0};
    double forbidden_angle_min_deg_{110.0};
    double forbidden_angle_max_deg_{160.0};

    State state_{State::MAP_GUIDED};
    int64_t enter_evidence_start_ns_{0};
    int64_t exit_evidence_start_ns_{0};
    int64_t recovery_enter_ns_{0};
    int64_t last_lidar_batch_ns_{std::numeric_limits<int64_t>::min() / 2};
    int64_t last_pose_solution_ns_{std::numeric_limits<int64_t>::min() / 2};
    size_t last_lidar_batch_size_{0};

    std::mutex mutex_;
    std::unordered_map<int64_t, TagObservation> camera_observations_;
    std::unordered_map<int64_t, int64_t> last_lidar_success_ns_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr camera_sub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoLidarRecoveryManager>());
    rclcpp::shutdown();
    return 0;
}
