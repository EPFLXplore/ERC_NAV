/*
 * Conceptual state machine: cube detection in an ArUco-guided LiDAR cloud
 *
 * STATE 0 - INITIALIZE
 *   Load the RANSAC, cube-geometry, gating, rate, and visualization parameters;
 *   create the TF listener; subscribe to the filtered LiDAR cloud and ArUco
 *   associations; and create the plane, center, and cube-result publishers.
 *
 * STATE 1 - WAIT FOR ASSOCIATIONS
 *   Cache each incoming set of allowed ArUco IDs, camera poses, and known map
 *   positions under a mutex. This message defines which landmark cubes may be
 *   searched for. Return to waiting for a filtered LiDAR cloud.
 *
 * STATE 2 - GATE A LIDAR CLOUD
 *   On a cloud callback, snapshot the current ArUco state. Reject the cloud if
 *   no association exists or if the configured processing-rate period has not
 *   elapsed. Otherwise read XYZ and marker_id fields and process each tag.
 *
 * STATE 3 - BUILD ONE TAG'S CANDIDATE CLOUD
 *   Transform the tag's known map position into the cloud frame when possible.
 *   Depending on the configured mode, use either the known map landmark or
 *   the camera-measured tag center as the expected range and bearing. Retain
 *   only points labeled with this ID and inside the radial/angular gates. If
 *   too few points remain, skip this tag and restart STATE 3 for the next tag.
 *
 * STATE 4 - EXTRACT AND CLASSIFY PLANES
 *   Repeatedly fit a RANSAC plane, remove its inliers, measure its dimensions
 *   and normal, and classify it as a plausible side or top cube face. Stop at
 *   the configured plane limit, when fitting fails, or when too few points
 *   remain. Optionally accept the strongest plane as a fallback. If no usable
 *   face remains, continue with the next tag.
 *
 * STATE 5 - GROUP FACES AND ESTIMATE THE CUBE CENTER
 *   Rank side and top candidates, select up to two nearby perpendicular side
 *   faces and a compatible top face, then infer the hidden cube volume. With
 *   two sides, extend their point bounding box toward the hidden faces; with
 *   one side, offset its centroid by half a cube width. Reject a center that
 *   lies too far from the expected landmark position.
 *
 * STATE 6 - TRANSFORM AND ACCUMULATE RESULTS
 *   Add accepted face inliers and visualization markers, transform the center
 *   into base_link, and append its marker ID, pose, and bearing to the outgoing
 *   cube message. If the transform is unavailable, keep visualization output
 *   in the cloud frame but omit that cube from the base_link result message.
 *   Continue at STATE 3 until every associated tag has been considered.
 *
 * STATE 7 - PUBLISH
 *   Publish plane markers, center markers, and the combined plane-inlier cloud.
 *   Publish the base_link cube message only when at least one center succeeded,
 *   then transition back to STATE 1/STATE 2 and wait for new asynchronous input.
 */
 #include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>
#include <mutex>
#include <limits>
#include <atomic>
#include <stdexcept>
#include <cstdio>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"


// #include <pcl/filters/project_inliers.h>


namespace {

inline double wrap180(double deg) {
    double d = fmod(deg + 180.0, 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
}

inline double angDeltaDeg(double a_deg, double b_deg) {
    return fabs(wrap180(b_deg - a_deg));
}

/* A point projected into the plane of a candidate face. The 2D basis is
 * arbitrary; every measurement below is invariant to how it was chosen. */
struct UV {
    double u{0.0};
    double v{0.0};
};

/* The oriented bounding rectangle of an in-plane point set. */
struct PlaneRect {
    double angle_rad{0.0};   // rectangle orientation within the (u, v) basis
    double extent_a{0.0};    // size along the rotated 'a' axis
    double extent_b{0.0};    // size along the rotated 'b' axis
    double mid_a{0.0};       // rectangle centre, rotated frame, relative to (u, v) origin
    double mid_b{0.0};

    double short_extent() const { return std::min(extent_a, extent_b); }
    double long_extent() const { return std::max(extent_a, extent_b); }
};

/* Reduce an in-plane point set to its "core": the points lying within
 * `core_radius` of the component-wise median. Returns the fraction of points
 * that were outside, i.e. how much of the plane's support does NOT sit in one
 * face-sized patch.
 *
 * Both halves matter, for the same reason. A RANSAC plane is infinite, so the
 * *correct* cube-face hypothesis is not clean either: extended downwards it
 * slices the ground at the same range, and that intersection line contributes
 * a long strip of coplanar inliers. Measuring a bounding box over the raw
 * inliers therefore reports the true face as metres wide. Trimming to the core
 * fixes the measurement; the outside fraction is what still distinguishes a
 * face (a dense patch plus a thin contaminating strip) from a ground or wall
 * plane (support spread far beyond any face-sized patch), which a bounding box
 * over the core alone can no longer tell apart.
 *
 * A disc rather than a box because the 2D basis is arbitrary: a box would make
 * the result depend on how the basis happened to be chosen. */
inline double trim_uv_to_core(std::vector<UV> &uv, double core_radius, UV &centre)
{
    if (uv.size() < 3 || !(core_radius > 0.0)) {
        return 0.0;
    }

    std::vector<double> scratch;
    scratch.reserve(uv.size());
    const size_t mid = uv.size() / 2;

    for (const auto &p : uv) {
        scratch.push_back(p.u);
    }
    std::nth_element(scratch.begin(), scratch.begin() + mid, scratch.end());
    const double median_u = scratch[mid];

    scratch.clear();
    for (const auto &p : uv) {
        scratch.push_back(p.v);
    }
    std::nth_element(scratch.begin(), scratch.begin() + mid, scratch.end());
    const double median_v = scratch[mid];

    centre.u = median_u;
    centre.v = median_v;

    const double total = static_cast<double>(uv.size());
    const double radius_sq = core_radius * core_radius;
    uv.erase(
        std::remove_if(
            uv.begin(), uv.end(),
            [median_u, median_v, radius_sq](const UV &p) {
                const double du = p.u - median_u;
                const double dv = p.v - median_v;
                return du * du + dv * dv > radius_sq;
            }),
        uv.end());

    return 1.0 - static_cast<double>(uv.size()) / total;
}

/* Minimum-area bounding rectangle, found by sweeping the rectangle
 * orientation over [0, pi/2) -- the full symmetry period of an axis-aligned
 * box. A sweep rather than 2D PCA because the cube top face is square: its
 * in-plane covariance is isotropic, so PCA's principal direction there is
 * numerically arbitrary, which is exactly the case that has to be measured
 * correctly. At a 5 deg step the area error on a square is under 0.5%, far
 * below LiDAR noise, for ~18 passes over a set of ~100 points. */
inline PlaneRect min_area_rect(const std::vector<UV> &uv, double angle_step_rad)
{
    PlaneRect best;
    if (uv.empty()) {
        return best;
    }
    if (!(angle_step_rad > 1e-6)) {
        angle_step_rad = 5.0 * M_PI / 180.0;
    }

    double best_area = std::numeric_limits<double>::infinity();
    for (double theta = 0.0; theta < M_PI_2; theta += angle_step_rad) {
        const double c = std::cos(theta);
        const double s = std::sin(theta);

        double min_a = std::numeric_limits<double>::infinity();
        double max_a = -std::numeric_limits<double>::infinity();
        double min_b = std::numeric_limits<double>::infinity();
        double max_b = -std::numeric_limits<double>::infinity();
        for (const auto &p : uv) {
            const double a = p.u * c + p.v * s;
            const double b = -p.u * s + p.v * c;
            min_a = std::min(min_a, a);
            max_a = std::max(max_a, a);
            min_b = std::min(min_b, b);
            max_b = std::max(max_b, b);
        }

        const double extent_a = max_a - min_a;
        const double extent_b = max_b - min_b;
        const double area = extent_a * extent_b;
        if (area < best_area) {
            best_area = area;
            best.angle_rad = theta;
            best.extent_a = extent_a;
            best.extent_b = extent_b;
            best.mid_a = 0.5 * (min_a + max_a);
            best.mid_b = 0.5 * (min_b + max_b);
        }
    }

    return best;
}

/* RANSAC plane model that only scores hypotheses whose inliers actually look
 * like a cube face.
 *
 * Stock SACSegmentation scores by inlier count alone, so a ground or wall
 * plane spanning the search cone outbids the face and the size prior only
 * gets applied afterwards, once the good hypothesis has already lost, been
 * subtracted from the remaining cloud, and burned one of the max_planes
 * slots. Folding the bound into the score means RANSAC can only ever win with
 * a face-shaped candidate.
 *
 * Only countWithinDistance() is overridden. getInliers() still comes from the
 * base selectWithinDistance(), i.e. the *untrimmed* plane inlier set -- which
 * is what the caller wants, since it subtracts those from the remaining cloud
 * and the strays should go with them. */
class BoundedPlaneModel : public pcl::SampleConsensusModelPlane<pcl::PointXYZ>
{
public:
    using Base = pcl::SampleConsensusModelPlane<pcl::PointXYZ>;
    using PointCloudConstPtr = typename Base::PointCloudConstPtr;

    BoundedPlaneModel(
        const PointCloudConstPtr &cloud,
        double max_short_extent_m,
        double max_long_extent_m,
        double core_radius_m,
        double max_outside_fraction,
        double angle_step_rad,
        const PointCloudConstPtr &context_cloud,
        int embedded_min_bin_points,
        int embedded_min_occupied_bins,
        bool require_vertical_long_axis,
        double vertical_long_axis_max_deg,
        double top_normal_z_min)
        : Base(cloud),
          max_short_extent_m_(max_short_extent_m),
          max_long_extent_m_(max_long_extent_m),
          core_radius_m_(core_radius_m),
          max_outside_fraction_(max_outside_fraction),
          angle_step_rad_(angle_step_rad),
          context_cloud_(context_cloud),
          embedded_min_bin_points_(embedded_min_bin_points),
          embedded_min_occupied_bins_(embedded_min_occupied_bins),
          require_vertical_long_axis_(require_vertical_long_axis),
          vertical_long_axis_min_cos_(
              std::cos(vertical_long_axis_max_deg * M_PI / 180.0)),
          top_normal_z_min_(top_normal_z_min)
    {
    }

    std::size_t countWithinDistance(
        const Eigen::VectorXf &coefficients,
        const double threshold) const override
    {
        if (coefficients.size() < 4) {
            return 0;
        }

        Eigen::Vector3f normal(coefficients[0], coefficients[1], coefficients[2]);
        const float normal_norm = normal.norm();
        if (normal_norm < 1e-9f) {
            return 0;
        }
        const float inv_norm = 1.0f / normal_norm;
        normal *= inv_norm;

        // Arbitrary orthonormal basis of the candidate plane. The measurement
        // below does not depend on which one we pick.
        Eigen::Vector3f reference(0.0f, 0.0f, 1.0f);
        if (std::fabs(normal.z()) > 0.9f) {
            reference = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        }
        Eigen::Vector3f u_axis = normal.cross(reference);
        if (u_axis.norm() < 1e-6f) {
            return 0;
        }
        u_axis.normalize();
        const Eigen::Vector3f v_axis = normal.cross(u_axis).normalized();

        std::vector<UV> uv;
        uv.reserve(this->indices_->size());
        for (const auto index : *this->indices_) {
            const auto &pt = (*this->input_)[index];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }

            const float distance =
                std::fabs(
                    coefficients[0] * pt.x +
                    coefficients[1] * pt.y +
                    coefficients[2] * pt.z +
                    coefficients[3]) * inv_norm;
            if (distance >= threshold) {
                continue;
            }

            const Eigen::Vector3f p(pt.x, pt.y, pt.z);
            uv.push_back({p.dot(u_axis), p.dot(v_axis)});
        }

        if (uv.empty()) {
            return 0;
        }

        UV centre;
        const double outside_fraction = trim_uv_to_core(uv, core_radius_m_, centre);
        if (uv.empty()) {
            return 0;
        }

        // A cube face's return is concentrated in one face-sized patch. A
        // ground or wall plane's is not, however face-sized its core happens
        // to look. This only sees the tightly gated candidate cloud, so it
        // catches sprawl but not a patch cropped down to face size -- that is
        // what the context check below is for.
        if (outside_fraction > max_outside_fraction_) {
            return 0;
        }

        const PlaneRect rect = min_area_rect(uv, angle_step_rad_);
        if (max_short_extent_m_ > 0.0 && rect.short_extent() > max_short_extent_m_) {
            return 0;
        }
        if (max_long_extent_m_ > 0.0 && rect.long_extent() > max_long_extent_m_) {
            return 0;
        }

        /* A cube standing on the ground has its side faces upright: the long
         * edge (cube_height_m) runs with gravity and the short edge
         * (cube_width_m) across it. Clutter fits have no such preference, so
         * requiring the long axis to be vertical throws out the tilted slivers
         * that otherwise measure face-sized.
         *
         * Skipped for near-horizontal planes: a top face's long axis is
         * horizontal by construction, so the test would reject every one.
         *
         * Note this is the cloud's z, as everywhere else in this node (see
         * top_vertical_dot_min), so it inherits any LiDAR mounting tilt. */
        if (require_vertical_long_axis_ &&
            std::fabs(normal.z()) < top_normal_z_min_) {
            const float cos_t = std::cos(static_cast<float>(rect.angle_rad));
            const float sin_t = std::sin(static_cast<float>(rect.angle_rad));
            const Eigen::Vector3f long_axis =
                rect.extent_a >= rect.extent_b
                    ? (u_axis * cos_t + v_axis * sin_t)
                    : (u_axis * -sin_t + v_axis * cos_t);
            if (std::fabs(long_axis.z()) < vertical_long_axis_min_cos_) {
                return 0;
            }
        }

        // Face-sized, but is it a face, or just a face-sized window onto
        // something bigger? Deliberately last: it is the only test that scans
        // a second cloud, and by here few hypotheses are left to scan for.
        if (is_embedded_in_larger_surface(
                normal, static_cast<float>(coefficients[3]) * inv_norm,
                u_axis, v_axis, centre, threshold)) {
            return 0;
        }

        // Score on the *core* count, so that among two admissible candidates
        // RANSAC prefers the more compact one rather than rating them equal.
        return uv.size();
    }

private:
    /* A cube face is a bounded object: its edges are real edges. Cut a
     * face-sized window out of a wall and it measures like a face, because the
     * candidate cloud has already been gated to a ~0.3 m radial band and a
     * cone around the landmark -- the rest of the wall was cropped away before
     * RANSAC ever saw it. So look again in the wider context cloud (the whole
     * landmark neighbourhood, ungated): does this plane keep finding coplanar
     * support well beyond the face?
     *
     * Counting that support is not enough on its own. A cube standing on the
     * ground has its own face plane clipping the ground along the base edge,
     * which contributes a long coplanar strip through the true face. The
     * difference is directional: a strip leaves support in ~two opposite
     * directions, whereas a window cut from a larger surface has support all
     * around it. So bin the outside support by bearing about the face centre
     * and reject on how many bins are occupied, not on how many points there
     * are. */
    bool is_embedded_in_larger_surface(
        const Eigen::Vector3f &normal,
        float plane_d,
        const Eigen::Vector3f &u_axis,
        const Eigen::Vector3f &v_axis,
        const UV &centre,
        const double threshold) const
    {
        if (!context_cloud_ || context_cloud_->empty() ||
            embedded_min_occupied_bins_ <= 0) {
            return false;
        }

        constexpr int kBins = 8;
        if (embedded_min_occupied_bins_ > kBins) {
            return false;
        }

        // The face centre, as a point on the plane: {u, v, n} is orthonormal
        // and every plane point satisfies p.n = -d.
        const Eigen::Vector3f face_centre =
            u_axis * static_cast<float>(centre.u) +
            v_axis * static_cast<float>(centre.v) -
            normal * plane_d;

        const double core_radius_sq = core_radius_m_ * core_radius_m_;
        int bin_counts[kBins] = {0};
        int occupied = 0;

        for (const auto &q : context_cloud_->points) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
                continue;
            }
            const Eigen::Vector3f p(q.x, q.y, q.z);
            if (std::fabs(normal.dot(p) + plane_d) >= threshold) {
                continue;
            }

            const Eigen::Vector3f rel = p - face_centre;
            const double du = rel.dot(u_axis);
            const double dv = rel.dot(v_axis);
            const double radius_sq = du * du + dv * dv;
            if (radius_sq <= core_radius_sq) {
                continue;   // part of the face patch itself
            }

            const double bearing = std::atan2(dv, du) + M_PI;   // [0, 2*pi)
            int bin = static_cast<int>(bearing * kBins / (2.0 * M_PI));
            bin = std::max(0, std::min(kBins - 1, bin));
            if (++bin_counts[bin] == embedded_min_bin_points_ &&
                ++occupied >= embedded_min_occupied_bins_) {
                // Verdict already decided; the rest of the scan cannot undo it.
                return true;
            }
        }

        return false;
    }

    double max_short_extent_m_;
    double max_long_extent_m_;
    double core_radius_m_;
    double max_outside_fraction_;
    double angle_step_rad_;
    PointCloudConstPtr context_cloud_;
    int embedded_min_bin_points_;
    int embedded_min_occupied_bins_;
    bool require_vertical_long_axis_;
    double vertical_long_axis_min_cos_;
    double top_normal_z_min_;
};
}

class DetectCubeNode : public rclcpp::Node {
public:
    DetectCubeNode()
        : rclcpp::Node("detect_cube_node"), tf_buffer_(this->get_clock()) {
        this->declare_parameter<double>("distance_threshold_inliers", 0.04);
        this->declare_parameter<int>("max_iterations", 1000);
        /* Bounded RANSAC: score plane hypotheses by how well their trimmed
         * inlier footprint fits a cube face, instead of by raw inlier count.
         * Disable to fall back to stock pcl::SACSegmentation. */
        this->declare_parameter<bool>("ransac_bounded_plane_enable", true);
        // 0 => derive from cube_width_m / cube_height_m + face_dimension_tolerance_m.
        this->declare_parameter<double>("ransac_max_plane_short_extent_m", 0.0);
        this->declare_parameter<double>("ransac_max_plane_long_extent_m", 0.0);
        /* Core radius, as a multiple of the face's circumscribed radius. The
         * core is what gets measured; everything outside it is treated as
         * coplanar clutter (typically the strip where the face plane, extended,
         * slices the ground). */
        this->declare_parameter<double>("ransac_core_radius_margin", 1.0);
        /* Max share of a hypothesis's inliers allowed to fall outside the core
         * before it is rejected as not-a-face. */
        this->declare_parameter<double>("ransac_max_outside_fraction", 0.35);
        // Orientation sweep step for the rectangle fit inside RANSAC scoring.
        this->declare_parameter<double>("ransac_bbox_angle_step_deg", 5.0);
        /* Reject a face-sized candidate that is really a window onto a larger
         * coplanar surface, by looking for coplanar support all around it in
         * the ungated landmark neighbourhood. */
        this->declare_parameter<bool>("ransac_embedded_veto_enable", true);
        // Coplanar points needed in a 45 deg bearing sector to count it occupied.
        this->declare_parameter<int>("ransac_embedded_min_bin_points", 3);
        // Occupied sectors (of 8) at which the candidate is called embedded.
        this->declare_parameter<int>("ransac_embedded_min_occupied_bins", 5);
        // Stride the context cloud down to this many points; 0 = no cap.
        this->declare_parameter<int>("ransac_context_max_points", 600);
        /* Require a side-face candidate's long edge to run with gravity, as a
         * cube standing on the ground does. Near-horizontal planes (top-face
         * candidates) are exempt -- their long axis is horizontal. */
        this->declare_parameter<bool>("ransac_require_vertical_long_axis", true);
        this->declare_parameter<double>("ransac_vertical_long_axis_max_deg", 20.0);
        this->declare_parameter<double>("t", 0.25);
        this->declare_parameter<int>("min_inliers", 40);
        this->declare_parameter<int>("max_lines", 4);
        this->declare_parameter<double>("cube_width_m", 0.25);
        this->declare_parameter<double>("cube_height_m", 0.32);
        this->declare_parameter<int>("max_planes", 8);
        this->declare_parameter<double>("plane_group_centroid_max_m", 0.35);
        this->declare_parameter<double>("face_dimension_tolerance_m", 0.08);
        this->declare_parameter<double>("side_perpendicular_dot_max", 0.35);
        this->declare_parameter<double>("top_vertical_dot_min", 0.65);
        this->declare_parameter<double>("face_min_short_frac", 0.25);
        this->declare_parameter<double>("side_min_long_frac", 0.25);
        this->declare_parameter<double>("top_min_long_frac", 0.25);
        this->declare_parameter<double>("face_score_tolerance_multiplier", 4.0);
        this->declare_parameter<double>("max_face_diagonal_multiplier", 1.8);
        this->declare_parameter<double>("max_center_error_m", 3.0);
        this->declare_parameter<bool>("accept_best_plane_fallback", false);
        this->declare_parameter<double>("process_rate_hz", 8.0);
        this->declare_parameter<double>("marker_lifetime_sec", 1.0);
        // Debug: publish every plane RANSAC returns (accepted or not) on
        // /visualization/ransac_candidate_planes, with dimensions and scores.
        this->declare_parameter<bool>("publish_candidate_plane_markers", true);
        // Half-width (m) of radial band: |r_point − r_expected| < tol, where r_expected is from map landmark → LiDAR frame.
        this->declare_parameter<double>("max_distance_from_aruco", 0.3);
        // Map frame for landmark_map_pos_* (same as lidar_phi_filter_node / multiview_aruco).
        this->declare_parameter<std::string>("map_frame", "map");
        // Angular gate vs ref_angle (camera bearing + offset); wide default while upstream filter is tight.
        this->declare_parameter<double>("angular_tolerance_deg", 45.0);
        this->declare_parameter<bool>("use_camera_aruco_position", false);
        this->declare_parameter<double>("camera_cone_half_angle_deg", 5.0);
        this->declare_parameter<double>("camera_cone_depth_tolerance_m", 1.0);
        this->declare_parameter<double>("camera_cone_depth_tolerance_ratio", 0.20);
        this->declare_parameter<double>("association_ambiguity_margin_m", 0.05);
        /* Segment midpoints closer than this (m): treat as duplicate hypotheses on one
         * feature (same place). Opposite face edges are ~t (~0.25 m) apart and never
         * enter here. If |cos θ| is high, refine one line (parallel duplicate); otherwise
         * keep the line whose centre bearing best matches ref_angle (perpendicular /
         * corner / noisy parallel RANSAC). */
        this->declare_parameter<double>("merge_duplicate_2d_line_mid_max_m", 0.10);
        this->declare_parameter<double>("merge_duplicate_2d_parallel_min_dir_dot", 0.96);
        /* Two 2D lines must look like opposite edges of the t×t face: nearly parallel
         * and midpoint spacing ~t. Otherwise RANSAC often fits clutter + edge; averaging
         * centres collapses bearing (e.g. toward +x). */
        this->declare_parameter<double>("opposite_2d_pair_min_dir_dot", 0.82);
        this->declare_parameter<double>("opposite_2d_mid_sep_min_frac", 0.40);
        this->declare_parameter<double>("opposite_2d_mid_sep_max_frac", 2.00);

        input_cloud_topic_ = "/lidar/points_near_camera_aruco_landmarks";
        output_cloud_topic_ = "/lidar/cube_plane_inliers";
        aruco_topic_ = "/perception/aruco_markers_for_lidar_association";

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        this->get_parameter("distance_threshold_inliers", distance_threshold_inliers);
        this->get_parameter("max_iterations", max_iterations_);
        this->get_parameter("t", t);
        this->get_parameter("min_inliers", min_inliers_);
        this->get_parameter("max_lines", max_lines_);
        this->get_parameter("cube_width_m", cube_width_m_);
        this->get_parameter("cube_height_m", cube_height_m_);
        this->get_parameter("max_planes", max_planes_);
        this->get_parameter("plane_group_centroid_max_m", plane_group_centroid_max_m_);
        this->get_parameter("face_dimension_tolerance_m", face_dimension_tolerance_m_);
        this->get_parameter("side_perpendicular_dot_max", side_perpendicular_dot_max_);
        this->get_parameter("top_vertical_dot_min", top_vertical_dot_min_);
        this->get_parameter("face_min_short_frac", face_min_short_frac_);
        this->get_parameter("side_min_long_frac", side_min_long_frac_);
        this->get_parameter("top_min_long_frac", top_min_long_frac_);
        this->get_parameter("face_score_tolerance_multiplier", face_score_tolerance_multiplier_);
        this->get_parameter("max_face_diagonal_multiplier", max_face_diagonal_multiplier_);
        this->get_parameter("max_center_error_m", max_center_error_m_);

        this->get_parameter(
            "ransac_bounded_plane_enable", ransac_bounded_plane_enable_);
        this->get_parameter(
            "ransac_max_plane_short_extent_m", ransac_max_plane_short_extent_m_);
        this->get_parameter(
            "ransac_max_plane_long_extent_m", ransac_max_plane_long_extent_m_);
        this->get_parameter(
            "ransac_core_radius_margin", ransac_core_radius_margin_);
        this->get_parameter(
            "ransac_max_outside_fraction", ransac_max_outside_fraction_);
        this->get_parameter(
            "ransac_bbox_angle_step_deg", ransac_bbox_angle_step_deg_);
        this->get_parameter(
            "ransac_embedded_veto_enable", ransac_embedded_veto_enable_);
        this->get_parameter(
            "ransac_embedded_min_bin_points", ransac_embedded_min_bin_points_);
        this->get_parameter(
            "ransac_embedded_min_occupied_bins", ransac_embedded_min_occupied_bins_);
        this->get_parameter(
            "ransac_context_max_points", ransac_context_max_points_);
        this->get_parameter(
            "ransac_require_vertical_long_axis", ransac_require_vertical_long_axis_);
        this->get_parameter(
            "ransac_vertical_long_axis_max_deg", ransac_vertical_long_axis_max_deg_);
        /* Resolve the extent bounds once: 0 means "derive from the cube
         * geometry", so the bounds track cube_width_m / cube_height_m without
         * having to be restated in the launch file. */
        if (ransac_max_plane_short_extent_m_ <= 0.0) {
            ransac_max_plane_short_extent_m_ =
                cube_width_m_ + face_dimension_tolerance_m_;
        }
        if (ransac_max_plane_long_extent_m_ <= 0.0) {
            ransac_max_plane_long_extent_m_ =
                std::max(cube_width_m_, cube_height_m_) + face_dimension_tolerance_m_;
        }
        /* Radius of the circle circumscribing the largest admissible face:
         * big enough that a whole face fits inside the core, small enough that
         * a ground/wall strip running through the face plane is cut off. */
        ransac_core_radius_m_ =
            ransac_core_radius_margin_ * 0.5 * std::hypot(
                ransac_max_plane_short_extent_m_, ransac_max_plane_long_extent_m_);

        this->get_parameter("accept_best_plane_fallback", accept_best_plane_fallback_);
        this->get_parameter("process_rate_hz", process_rate_hz_);
        this->get_parameter("marker_lifetime_sec", marker_lifetime_sec_);
        this->get_parameter(
            "publish_candidate_plane_markers", publish_candidate_plane_markers_);
        this->get_parameter("max_distance_from_aruco", max_distance_from_aruco_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("angular_tolerance_deg", angular_tolerance_deg_);
        bool initial_camera_mode = false;
        this->get_parameter("use_camera_aruco_position", initial_camera_mode);
        use_camera_aruco_position_.store(initial_camera_mode);
        this->get_parameter("camera_cone_half_angle_deg", camera_cone_half_angle_deg_);
        this->get_parameter(
            "camera_cone_depth_tolerance_m", camera_cone_depth_tolerance_m_);
        this->get_parameter(
            "camera_cone_depth_tolerance_ratio", camera_cone_depth_tolerance_ratio_);
        this->get_parameter(
            "association_ambiguity_margin_m", association_ambiguity_margin_m_);
        this->get_parameter("merge_duplicate_2d_line_mid_max_m",
            merge_duplicate_2d_line_mid_max_m_);
        this->get_parameter("merge_duplicate_2d_parallel_min_dir_dot",
            merge_duplicate_2d_parallel_min_dir_dot_);
        this->get_parameter("opposite_2d_pair_min_dir_dot",
            opposite_2d_pair_min_dir_dot_);
        this->get_parameter("opposite_2d_mid_sep_min_frac",
            opposite_2d_mid_sep_min_frac_);
        this->get_parameter("opposite_2d_mid_sep_max_frac",
            opposite_2d_mid_sep_max_frac_);
        if (!(camera_cone_half_angle_deg_ > 0.0) ||
            !(camera_cone_half_angle_deg_ < 90.0) ||
            !(camera_cone_depth_tolerance_m_ > 0.0) ||
            !(camera_cone_depth_tolerance_ratio_ >= 0.0)) {
            throw std::runtime_error("invalid camera recovery cone parameters");
        }
        min_process_period_ns_ =
            process_rate_hz_ > 0.0 ? static_cast<int64_t>(1e9 / process_rate_hz_) : 0;
        last_process_time_ns_ = 0;
        full_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
                detect_lignes(*message);
            }
        );

        lines_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/detected_cube_planes", qos);
        centre_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/cube_centers", qos);
        candidate_planes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization/ransac_candidate_planes", qos);
        cube_markers_pub_ = create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("/perception/lidar_cube_markers", qos);

        aruco_subscriber_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            aruco_topic_, qos,
            [this](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                on_aruco_markers(msg);
            }
        );
        auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        mode_subscriber_ = create_subscription<std_msgs::msg::Bool>(
            "/perception/use_camera_aruco_position", mode_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                set_camera_mode(msg->data, "recovery supervisor");
                if (get_parameter("use_camera_aruco_position").as_bool() != msg->data) {
                    const auto result = set_parameter(rclcpp::Parameter(
                        "use_camera_aruco_position", msg->data));
                    if (!result.successful) {
                        RCLCPP_ERROR(get_logger(),
                            "Failed to mirror recovery mode into parameter: %s",
                            result.reason.c_str());
                    }
                }
            });
        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto &parameter : parameters) {
                    if (parameter.get_name() == "use_camera_aruco_position") {
                        set_camera_mode(parameter.as_bool(), "parameter update");
                    }
                }
                return result;
            });
        //RCLCPP_INFO(this->get_logger(), "initialized detect_cube_node");

    }

private:
    void set_camera_mode(bool enabled, const char *source)
    {
        const bool previous = use_camera_aruco_position_.exchange(enabled);
        if (previous == enabled) return;

        {
            std::lock_guard<std::mutex> lock(aruco_mutex_);
            aruco_ids_.clear();
            aruco_poses_.clear();
            aruco_landmark_map_x_.clear();
            aruco_landmark_map_y_.clear();
            aruco_header_ = std_msgs::msg::Header();
        }
        // Allow the filter to publish a cloud and a fresh association made in
        // the new mode before accepting another detector input.
        mode_switch_guard_until_ns_.store(
            now().nanoseconds() + static_cast<int64_t>(0.25 * 1e9));
        RCLCPP_WARN(get_logger(),
            "Cube detector mode changed to %s by %s; cached associations cleared",
            enabled ? "CAMERA_GUIDED" : "MAP_GUIDED", source);
    }

    void on_aruco_markers(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(aruco_mutex_);
        aruco_ids_ = msg->marker_ids;
        aruco_poses_ = msg->poses;
        aruco_landmark_map_x_ = msg->landmark_map_pos_x;
        aruco_landmark_map_y_ = msg->landmark_map_pos_y;
        aruco_header_ = msg->header;

        // RCLCPP_INFO(this->get_logger(),
        //    "[detect_cube INPUT] received aruco_markers: ids=%zu poses=%zu map_x=%zu map_y=%zu",
        //    aruco_ids_.size(), aruco_poses_.size(),
        //    aruco_landmark_map_x_.size(), aruco_landmark_map_y_.size());
    }

    // Converts map coordinates to cloud frame coordinates and returns the horizontal range in the cloud frame
    bool expected_horizontal_range_from_map(
        double map_x,
        double map_y,
        const std::string &cloud_frame_id,
        double &out_range_xy)
    {
        double dummy_x = 0.0;
        double dummy_y = 0.0;
        double dummy_bearing_deg = 0.0;
        builtin_interfaces::msg::Time zero_stamp;
        return expected_landmark_in_cloud_frame(
            map_x, map_y, cloud_frame_id, zero_stamp,
            dummy_x, dummy_y, out_range_xy, dummy_bearing_deg);
    }

    // Transforms a map landmark into the cloud frame and reports its (x, y), planar range
    // and bearing (deg, atan2(y, x)) all expressed in the cloud frame.
    bool expected_landmark_in_cloud_frame(
        double map_x,
        double map_y,
        const std::string &cloud_frame_id,
        const builtin_interfaces::msg::Time &cloud_stamp,
        double &out_x,
        double &out_y,
        double &out_range_xy,
        double &out_bearing_deg)
    {
        geometry_msgs::msg::PointStamped pin;
        pin.header.frame_id = map_frame_;
        pin.header.stamp = rclcpp::Time(0);
        pin.point.x = map_x;
        pin.point.y = map_y;
        pin.point.z = 0.0;
        geometry_msgs::msg::TransformStamped tf;
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1))) {
                    return false;
                }
                tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
            } else {
                const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
                if (tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, t, rclcpp::Duration::from_seconds(0.1))) {
                    tf = tf_buffer_.lookupTransform(
                        cloud_frame_id, map_frame_, t, rclcpp::Duration::from_seconds(0.1));
                } else {
                    if (!tf_buffer_.canTransform(
                            cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                        return false;
                    }
                    tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
                }
            }
            geometry_msgs::msg::PointStamped pout;
            tf2::doTransform(pin, pout, tf);
            out_x = pout.point.x;
            out_y = pout.point.y;
            out_range_xy = std::hypot(out_x, out_y);
            out_bearing_deg = std::atan2(out_y, out_x) * 180.0 / M_PI;
            return true;
        } catch (const tf2::TransformException &ex) {
            (void)ex;
            try {
                if (!tf_buffer_.canTransform(
                        cloud_frame_id, map_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
                    return false;
                }
                tf = tf_buffer_.lookupTransform(cloud_frame_id, map_frame_, tf2::TimePointZero);
                geometry_msgs::msg::PointStamped pout;
                tf2::doTransform(pin, pout, tf);
                out_x = pout.point.x;
                out_y = pout.point.y;
                out_range_xy = std::hypot(out_x, out_y);
                out_bearing_deg = std::atan2(out_y, out_x) * 180.0 / M_PI;
                return true;
            } catch (const tf2::TransformException &) {
                return false;
            }
        }
    }

    bool observed_aruco_in_cloud_frame(
        const geometry_msgs::msg::Point &position,
        const std::string &source_frame,
        const std::string &cloud_frame,
        const builtin_interfaces::msg::Time &cloud_stamp,
        double &out_x,
        double &out_y,
        double &out_range_xy,
        double &out_bearing_deg)
    {
        if (source_frame.empty() || cloud_frame.empty() ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            return false;
        }

        geometry_msgs::msg::PointStamped point_in;
        point_in.header.frame_id = source_frame;
        point_in.header.stamp = cloud_stamp;
        point_in.point = position;

        geometry_msgs::msg::PointStamped point_out;
        try {
            const bool zero_stamp =
                (cloud_stamp.sec == 0u && cloud_stamp.nanosec == 0u);
            geometry_msgs::msg::TransformStamped transform;
            if (zero_stamp) {
                if (!tf_buffer_.canTransform(
                        cloud_frame, source_frame, tf2::TimePointZero,
                        tf2::durationFromSec(0.1))) {
                    return false;
                }
                transform = tf_buffer_.lookupTransform(
                    cloud_frame, source_frame, tf2::TimePointZero);
            } else {
                const rclcpp::Time t(cloud_stamp, get_clock()->get_clock_type());
                if (tf_buffer_.canTransform(
                        cloud_frame, source_frame, t,
                        rclcpp::Duration::from_seconds(0.1))) {
                    transform = tf_buffer_.lookupTransform(
                        cloud_frame, source_frame, t,
                        rclcpp::Duration::from_seconds(0.1));
                } else {
                    if (!tf_buffer_.canTransform(
                            cloud_frame, source_frame, tf2::TimePointZero,
                            tf2::durationFromSec(0.05))) {
                        return false;
                    }
                    transform = tf_buffer_.lookupTransform(
                        cloud_frame, source_frame, tf2::TimePointZero);
                }
            }
            tf2::doTransform(point_in, point_out, transform);
        } catch (const tf2::TransformException &) {
            return false;
        }

        out_x = point_out.point.x;
        out_y = point_out.point.y;
        out_range_xy = std::hypot(out_x, out_y);
        out_bearing_deg = std::atan2(out_y, out_x) * 180.0 / M_PI;
        return std::isfinite(out_x) && std::isfinite(out_y) &&
            std::isfinite(out_range_xy) && std::isfinite(out_bearing_deg);
    }

    static bool is_invalid_landmark_xy(double mx, double my)
    {
        constexpr double kSentinel = 999999.0;
        return mx >= kSentinel - 1.0 || my >= kSentinel - 1.0;
    }

    struct Vec3 {
        double x{0.0};
        double y{0.0};
        double z{0.0};
    };

    struct PlaneDetection {
        Vec3 normal;
        Vec3 centroid;
        Vec3 u_axis;
        Vec3 v_axis;
        double d{0.0};
        double extent_u{0.0};
        double extent_v{0.0};
        double dim_short{0.0};
        double dim_long{0.0};
        double side_score{std::numeric_limits<double>::infinity()};
        double top_score{std::numeric_limits<double>::infinity()};
        bool is_side{false};
        bool is_top{false};
        // Why the plane was not kept as a cube face ("" when accepted).
        std::string reject_reason;
        size_t inlier_count{0};
        pcl::PointIndices::Ptr indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
    };

    static Vec3 add_vec(const Vec3 &a, const Vec3 &b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    static Vec3 sub_vec(const Vec3 &a, const Vec3 &b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    static Vec3 scale_vec(const Vec3 &a, double s)
    {
        return {a.x * s, a.y * s, a.z * s};
    }

    static double dot_vec(const Vec3 &a, const Vec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vec3 cross_vec(const Vec3 &a, const Vec3 &b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    static double norm_vec(const Vec3 &a)
    {
        return std::sqrt(dot_vec(a, a));
    }

    static Vec3 normalize_vec(const Vec3 &a)
    {
        const double n = norm_vec(a);
        if (n < 1e-9) {
            return {0.0, 0.0, 0.0};
        }
        return scale_vec(a, 1.0 / n);
    }

    static geometry_msgs::msg::Point to_point_msg(const Vec3 &v)
    {
        geometry_msgs::msg::Point p;
        p.x = v.x;
        p.y = v.y;
        p.z = v.z;
        return p;
    }

    bool transform_point_to_base(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        geometry_msgs::msg::Point &point,
        std::string &target_frame)
    {
        target_frame = cloud_msg.header.frame_id;
        geometry_msgs::msg::PointStamped point_in;
        point_in.header.frame_id = cloud_msg.header.frame_id;
        point_in.header.stamp = rclcpp::Time(0);
        point_in.point = point;

        try {
            if (tf_buffer_.canTransform(
                    "base_link",
                    cloud_msg.header.frame_id,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.05))) {
                const auto transform = tf_buffer_.lookupTransform(
                    "base_link",
                    cloud_msg.header.frame_id,
                    tf2::TimePointZero);
                geometry_msgs::msg::PointStamped point_out;
                tf2::doTransform(point_in, point_out, transform);
                point = point_out.point;
                target_frame = "base_link";
                return true;
            }

            if (tf_buffer_.canTransform(
                    "Service_Module_v5_1",
                    cloud_msg.header.frame_id,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.05)) &&
                tf_buffer_.canTransform(
                    "base_link",
                    "Service_Module_v5_1",
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.05))) {
                const auto transform_lidar_to_service = tf_buffer_.lookupTransform(
                    "Service_Module_v5_1",
                    cloud_msg.header.frame_id,
                    tf2::TimePointZero);
                const auto transform_service_to_base = tf_buffer_.lookupTransform(
                    "base_link",
                    "Service_Module_v5_1",
                    tf2::TimePointZero);

                geometry_msgs::msg::PointStamped point_service;
                geometry_msgs::msg::PointStamped point_base;
                tf2::doTransform(point_in, point_service, transform_lidar_to_service);
                tf2::doTransform(point_service, point_base, transform_service_to_base);
                point = point_base.point;
                target_frame = "base_link";
                return true;
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "[detect_cube PLANES] transform to base_link failed: %s", ex.what());
        }

        return false;
    }

    /* RANSAC with the cube-face size prior folded into the hypothesis score,
     * via BoundedPlaneModel. pcl::RandomSampleConsensus has no equivalent of
     * SACSegmentation's setOptimizeCoefficients(true), so the refit is done
     * here by hand -- and re-scored afterwards, because a least-squares refit
     * over the inliers can perfectly well tilt an admissible plane into an
     * oversized one. */
    bool fit_bounded_plane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &remaining,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &context_cloud,
        int min_inliers,
        pcl::PointIndices &inliers,
        pcl::ModelCoefficients &coefficients)
    {
        auto model = pcl::SampleConsensusModel<pcl::PointXYZ>::Ptr(
            new BoundedPlaneModel(
                remaining,
                ransac_max_plane_short_extent_m_,
                ransac_max_plane_long_extent_m_,
                ransac_core_radius_m_,
                ransac_max_outside_fraction_,
                ransac_bbox_angle_step_deg_ * M_PI / 180.0,
                ransac_embedded_veto_enable_
                    ? context_cloud
                    : pcl::PointCloud<pcl::PointXYZ>::ConstPtr(),
                ransac_embedded_min_bin_points_,
                ransac_embedded_min_occupied_bins_,
                ransac_require_vertical_long_axis_,
                ransac_vertical_long_axis_max_deg_,
                top_vertical_dot_min_));

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(distance_threshold_inliers);
        ransac.setMaxIterations(max_iterations_);

        if (!ransac.computeModel()) {
            /* Every hypothesis scored zero: no minimal sample produced a
             * face-sized plane. Expected on clutter, but if this is frequent
             * on frames that should contain a cube, the extent bounds or
             * ransac_extent_trim_margin are too tight. */
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "[detect_cube PLANES] bounded RANSAC found no face-sized plane "
                "in %zu points (short<=%.3f long<=%.3f core_r=%.3f outside<=%.2f)",
                remaining->size(),
                ransac_max_plane_short_extent_m_,
                ransac_max_plane_long_extent_m_,
                ransac_core_radius_m_,
                ransac_max_outside_fraction_);
            return false;
        }

        pcl::Indices ransac_inliers;
        ransac.getInliers(ransac_inliers);

        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        if (ransac_inliers.size() < static_cast<size_t>(min_inliers) ||
            coeff.size() < 4) {
            return false;
        }

        Eigen::VectorXf optimized_coeff;
        model->optimizeModelCoefficients(ransac_inliers, coeff, optimized_coeff);

        pcl::Indices optimized_inliers;
        model->selectWithinDistance(
            optimized_coeff, distance_threshold_inliers, optimized_inliers);

        // Keep the refit only if it stayed inside the size bounds.
        const std::size_t optimized_score =
            model->countWithinDistance(optimized_coeff, distance_threshold_inliers);
        if (optimized_score >= static_cast<size_t>(min_inliers)) {
            coeff = optimized_coeff;
            ransac_inliers = std::move(optimized_inliers);
        }

        inliers.indices = std::move(ransac_inliers);
        coefficients.values.assign(coeff.data(), coeff.data() + coeff.size());
        return true;
    }

    bool extract_plane(
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &context_cloud,
        int min_inliers,
        PlaneDetection &plane)
    {
        if (remaining->size() < static_cast<size_t>(min_inliers)) {
            return false;
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        if (ransac_bounded_plane_enable_) {
            if (!fit_bounded_plane(
                    remaining, context_cloud, min_inliers, *inliers, *coefficients)) {
                return false;
            }
        } else {
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(distance_threshold_inliers);
            seg.setMaxIterations(max_iterations_);
            seg.setInputCloud(remaining);
            seg.segment(*inliers, *coefficients);
        }

        if (inliers->indices.size() < static_cast<size_t>(min_inliers) ||
            coefficients->values.size() < 4) {
            return false;
        }
        plane.indices = inliers;

        Vec3 normal{
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2]
        };
        const double n = norm_vec(normal);
        if (n < 1e-9) {
            return false;
        }
        normal = scale_vec(normal, 1.0 / n);
        const double d = coefficients->values[3] / n;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.setNegative(false);
        plane.inliers.reset(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*plane.inliers);
        plane.inlier_count = plane.inliers->size();
        if (plane.inlier_count < static_cast<size_t>(min_inliers)) {
            return false;
        }

        Vec3 centroid;
        for (const auto &pt : plane.inliers->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            centroid.x += pt.x;
            centroid.y += pt.y;
            centroid.z += pt.z;
        }
        centroid = scale_vec(centroid, 1.0 / static_cast<double>(plane.inlier_count));

        Vec3 reference{0.0, 0.0, 1.0};
        if (std::fabs(dot_vec(normal, reference)) > 0.9) {
            reference = {1.0, 0.0, 0.0};
        }
        Vec3 u_axis = normalize_vec(cross_vec(normal, reference));
        if (norm_vec(u_axis) < 1e-6) {
            u_axis = normalize_vec(cross_vec(normal, {0.0, 1.0, 0.0}));
        }
        Vec3 v_axis = normalize_vec(cross_vec(normal, u_axis));

        /* (u_axis, v_axis) above is an *arbitrary* in-plane frame: for a
         * vertical side face it happens to come out gravity-aligned, but for
         * a top face the normal is near-vertical, so it gets pinned to the
         * LiDAR x/y axes instead of the cube's yaw. Measuring a bounding box
         * directly in it therefore reports a 0.25 x 0.25 top face yawed 45 deg
         * as 0.354 x 0.354. Use it only to project to 2D, then let
         * min_area_rect() recover the true edge-aligned extents. */
        std::vector<UV> uv;
        uv.reserve(plane.inliers->size());
        for (const auto &pt : plane.inliers->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            const Vec3 rel = sub_vec({pt.x, pt.y, pt.z}, centroid);
            uv.push_back({dot_vec(rel, u_axis), dot_vec(rel, v_axis)});
        }
        if (uv.empty()) {
            return false;
        }
        UV core_centre;
        trim_uv_to_core(uv, ransac_core_radius_m_, core_centre);
        if (uv.empty()) {
            return false;
        }

        /* Finer sweep than the one inside RANSAC scoring: this runs once per
         * accepted plane rather than once per hypothesis, and its output feeds
         * the side/top size gating below. */
        constexpr double kMeasureAngleStepRad = 1.0 * M_PI / 180.0;
        const PlaneRect rect = min_area_rect(uv, kMeasureAngleStepRad);

        /* Re-express the in-plane frame along the rectangle's own edges and
         * recentre on the rectangle, so the debug markers drawn from
         * (centroid, u_axis, v_axis, extent_u, extent_v) match the dimensions
         * reported here. */
        const double cos_t = std::cos(rect.angle_rad);
        const double sin_t = std::sin(rect.angle_rad);
        const Vec3 a_axis = add_vec(
            scale_vec(u_axis, cos_t), scale_vec(v_axis, sin_t));
        const Vec3 b_axis = add_vec(
            scale_vec(u_axis, -sin_t), scale_vec(v_axis, cos_t));
        centroid = add_vec(
            centroid,
            add_vec(scale_vec(a_axis, rect.mid_a), scale_vec(b_axis, rect.mid_b)));

        plane.normal = normal;
        plane.centroid = centroid;
        plane.u_axis = a_axis;
        plane.v_axis = b_axis;
        plane.d = d;
        plane.extent_u = std::max(0.0, rect.extent_a);
        plane.extent_v = std::max(0.0, rect.extent_b);
        plane.dim_short = std::min(plane.extent_u, plane.extent_v);
        plane.dim_long = std::max(plane.extent_u, plane.extent_v);

        const double side_score =
            std::fabs(plane.dim_short - cube_width_m_) +
            std::fabs(plane.dim_long - cube_height_m_);
        const double top_score =
            std::fabs(plane.dim_short - cube_width_m_) +
            std::fabs(plane.dim_long - cube_width_m_);
        const bool plausible_side_size =
            plane.dim_short >= face_min_short_frac_ * cube_width_m_ &&
            plane.dim_short <= cube_width_m_ + face_dimension_tolerance_m_ &&
            plane.dim_long >= side_min_long_frac_ * cube_height_m_ &&
            plane.dim_long <= cube_height_m_ + face_dimension_tolerance_m_;
        const bool plausible_top_size =
            plane.dim_short >= face_min_short_frac_ * cube_width_m_ &&
            plane.dim_short <= cube_width_m_ + face_dimension_tolerance_m_ &&
            plane.dim_long >= top_min_long_frac_ * cube_width_m_ &&
            plane.dim_long <= cube_width_m_ + face_dimension_tolerance_m_;
        const double face_diagonal = std::hypot(cube_width_m_, cube_height_m_);
        const double max_face_extent =
            max_face_diagonal_multiplier_ > 0.0
                ? max_face_diagonal_multiplier_ * face_diagonal
                : std::numeric_limits<double>::infinity();
        const bool plausible_face_extent = plane.dim_long <= max_face_extent;

        plane.side_score = side_score;
        plane.top_score = top_score;
        plane.is_side = plausible_side_size &&
            plausible_face_extent &&
            side_score <= face_score_tolerance_multiplier_ * face_dimension_tolerance_m_;
        plane.is_top =
            plausible_top_size &&
            plausible_face_extent &&
            top_score <= face_score_tolerance_multiplier_ * face_dimension_tolerance_m_ &&
            (std::fabs(plane.normal.z) >= top_vertical_dot_min_ ||
             top_score + 0.03 < side_score);

        plane.reject_reason.clear();
        if (!plane.is_side && !plane.is_top) {
            if (!plausible_face_extent) {
                plane.reject_reason = "extent";
            } else if (!plausible_side_size && !plausible_top_size) {
                plane.reject_reason = "dims";
            } else if (plausible_top_size &&
                       std::fabs(plane.normal.z) < top_vertical_dot_min_) {
                plane.reject_reason = "score/tilt";
            } else {
                plane.reject_reason = "score";
            }
        }

        return true;
    }

    void append_plane_markers(
        const PlaneDetection &plane,
        int marker_id,
        const std_msgs::msg::Header &header,
        visualization_msgs::msg::MarkerArray &markers)
    {
        const Vec3 du = scale_vec(plane.u_axis, plane.extent_u * 0.5);
        const Vec3 dv = scale_vec(plane.v_axis, plane.extent_v * 0.5);
        const Vec3 c = plane.centroid;
        const Vec3 corners[4] = {
            add_vec(add_vec(c, du), dv),
            add_vec(sub_vec(c, du), dv),
            sub_vec(sub_vec(c, du), dv),
            sub_vec(add_vec(c, du), dv)
        };

        visualization_msgs::msg::Marker outline;
        outline.header = header;
        outline.ns = plane.is_top ? "detected_cube_top_planes" : "detected_cube_side_planes";
        outline.id = marker_id;
        outline.type = visualization_msgs::msg::Marker::LINE_LIST;
        outline.action = visualization_msgs::msg::Marker::ADD;
        outline.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        outline.scale.x = 0.015;
        outline.color.r = plane.is_top ? 0.1f : 0.0f;
        outline.color.g = plane.is_top ? 0.4f : 0.8f;
        outline.color.b = plane.is_top ? 1.0f : 0.2f;
        outline.color.a = 1.0f;
        for (int i = 0; i < 4; ++i) {
            outline.points.push_back(to_point_msg(corners[i]));
            outline.points.push_back(to_point_msg(corners[(i + 1) % 4]));
        }
        markers.markers.push_back(outline);

        visualization_msgs::msg::Marker normal_marker;
        normal_marker.header = header;
        normal_marker.ns = "detected_cube_plane_normals";
        normal_marker.id = marker_id + 1000;
        normal_marker.type = visualization_msgs::msg::Marker::ARROW;
        normal_marker.action = visualization_msgs::msg::Marker::ADD;
        normal_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        normal_marker.scale.x = 0.015;
        normal_marker.scale.y = 0.03;
        normal_marker.scale.z = 0.04;
        normal_marker.color.r = 1.0f;
        normal_marker.color.g = 0.8f;
        normal_marker.color.b = 0.0f;
        normal_marker.color.a = 1.0f;
        normal_marker.points.push_back(to_point_msg(plane.centroid));
        normal_marker.points.push_back(to_point_msg(add_vec(
            plane.centroid, scale_vec(plane.normal, 0.18))));
        markers.markers.push_back(normal_marker);
    }

    // Debug visualization of every plane RANSAC returned for one tag, before
    // and independently of the side/top selection logic. Accepted planes are
    // drawn in green (side) or blue (top), rejected ones in red, each labelled
    // with its inlier count, measured dimensions and face scores.
    void append_candidate_plane_markers(
        const PlaneDetection &plane,
        int64_t aruco_id,
        int plane_idx,
        const std_msgs::msg::Header &header,
        visualization_msgs::msg::MarkerArray &markers)
    {
        const int marker_id = static_cast<int>(
            (static_cast<int64_t>(aruco_id) * 100 + plane_idx) & 0x7fffffff);

        std_msgs::msg::ColorRGBA color;
        color.a = 1.0f;
        if (plane.is_top) {
            color.r = 0.1f;
            color.g = 0.4f;
            color.b = 1.0f;
        } else if (plane.is_side) {
            color.r = 0.0f;
            color.g = 0.9f;
            color.b = 0.2f;
        } else {
            color.r = 1.0f;
            color.g = 0.15f;
            color.b = 0.1f;
        }

        const Vec3 du = scale_vec(plane.u_axis, plane.extent_u * 0.5);
        const Vec3 dv = scale_vec(plane.v_axis, plane.extent_v * 0.5);
        const Vec3 c = plane.centroid;
        const Vec3 corners[4] = {
            add_vec(add_vec(c, du), dv),
            add_vec(sub_vec(c, du), dv),
            sub_vec(sub_vec(c, du), dv),
            sub_vec(add_vec(c, du), dv)
        };

        visualization_msgs::msg::Marker outline;
        outline.header = header;
        outline.ns = "ransac_candidate_outlines";
        outline.id = marker_id;
        outline.type = visualization_msgs::msg::Marker::LINE_LIST;
        outline.action = visualization_msgs::msg::Marker::ADD;
        outline.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        outline.scale.x = 0.01;
        outline.color = color;
        for (int i = 0; i < 4; ++i) {
            outline.points.push_back(to_point_msg(corners[i]));
            outline.points.push_back(to_point_msg(corners[(i + 1) % 4]));
        }
        markers.markers.push_back(outline);

        visualization_msgs::msg::Marker normal_marker;
        normal_marker.header = header;
        normal_marker.ns = "ransac_candidate_normals";
        normal_marker.id = marker_id;
        normal_marker.type = visualization_msgs::msg::Marker::ARROW;
        normal_marker.action = visualization_msgs::msg::Marker::ADD;
        normal_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        normal_marker.scale.x = 0.01;
        normal_marker.scale.y = 0.02;
        normal_marker.scale.z = 0.03;
        normal_marker.color = color;
        normal_marker.points.push_back(to_point_msg(plane.centroid));
        normal_marker.points.push_back(to_point_msg(add_vec(
            plane.centroid, scale_vec(plane.normal, 0.12))));
        markers.markers.push_back(normal_marker);

        if (plane.inliers && !plane.inliers->empty()) {
            visualization_msgs::msg::Marker points;
            points.header = header;
            points.ns = "ransac_candidate_inliers";
            points.id = marker_id;
            points.type = visualization_msgs::msg::Marker::POINTS;
            points.action = visualization_msgs::msg::Marker::ADD;
            points.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
            points.scale.x = 0.02;
            points.scale.y = 0.02;
            points.color = color;
            points.color.a = 0.85f;
            points.pose.orientation.w = 1.0;
            points.points.reserve(plane.inliers->size());
            for (const auto &pt : plane.inliers->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }
                points.points.push_back(to_point_msg({pt.x, pt.y, pt.z}));
            }
            markers.markers.push_back(points);
        }

        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "ransac_candidate_labels";
        label.id = marker_id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        label.scale.z = 0.05;
        label.color = color;
        label.pose.orientation.w = 1.0;
        label.pose.position = to_point_msg(add_vec(
            plane.centroid, {0.0, 0.0, 0.5 * plane.extent_v + 0.06}));
        char text[256];
        std::snprintf(
            text, sizeof(text),
            "id=%ld p%d %s\nn=%zu short=%.3f long=%.3f\nside=%.3f top=%.3f nz=%.2f",
            static_cast<long>(aruco_id), plane_idx,
            plane.is_top ? "TOP" : (plane.is_side ? "SIDE" : plane.reject_reason.c_str()),
            plane.inlier_count, plane.dim_short, plane.dim_long,
            plane.side_score, plane.top_score, plane.normal.z);
        label.text = text;
        markers.markers.push_back(label);
    }

    void detect_planes(const sensor_msgs::msg::PointCloud2 &cloud_msg)
    {
        const int64_t now_ns = this->now().nanoseconds();
        if (now_ns < mode_switch_guard_until_ns_.load()) return;
        const bool use_camera_mode = use_camera_aruco_position_.load();

        std::vector<int64_t> aruco_ids;
        std::vector<geometry_msgs::msg::Pose> aruco_poses;
        std::vector<double> aruco_landmark_map_x;
        std::vector<double> aruco_landmark_map_y;
        std_msgs::msg::Header aruco_header;
        {
            std::lock_guard<std::mutex> lk(aruco_mutex_);
            aruco_ids = aruco_ids_;
            aruco_poses = aruco_poses_;
            aruco_landmark_map_x = aruco_landmark_map_x_;
            aruco_landmark_map_y = aruco_landmark_map_y_;
            aruco_header = aruco_header_;
        }
        const size_t association_count = std::min(aruco_ids.size(), aruco_poses.size());
        if (association_count == 0) {
            return;
        }

        if (now_ns - last_process_time_ns_ < min_process_period_ns_) {
            return;
        }
        last_process_time_ns_ = now_ns;

        const auto marker_id_field = std::find_if(
            cloud_msg.fields.begin(), cloud_msg.fields.end(),
            [](const sensor_msgs::msg::PointField &field) {
                return field.name == "marker_id";
            });
        if (marker_id_field == cloud_msg.fields.end() ||
            marker_id_field->datatype != sensor_msgs::msg::PointField::INT32) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "[detect_cube] filtered cloud has no INT32 marker_id field");
            return;
        }

        full_cloud_->clear();
        std::vector<int32_t> point_marker_ids;
        const size_t point_count =
            static_cast<size_t>(cloud_msg.width) * static_cast<size_t>(cloud_msg.height);
        full_cloud_->reserve(point_count);
        point_marker_ids.reserve(point_count);
        sensor_msgs::PointCloud2ConstIterator<int32_t> id_it(cloud_msg, "marker_id");
        for (sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud_msg, "x"),
                                                            y_it(cloud_msg, "y"),
                                                            z_it(cloud_msg, "z");
             x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++id_it) {
            full_cloud_->push_back(pcl::PointXYZ(*x_it, *y_it, *z_it));
            point_marker_ids.push_back(*id_it);
        }

        auto expected_position_for_index =
            [&](size_t index, double &out_x, double &out_y,
                double &out_range, double &out_bearing) {
                if (index >= association_count) {
                    return false;
                }
                if (use_camera_mode) {
                    return observed_aruco_in_cloud_frame(
                        aruco_poses[index].position, aruco_header.frame_id,
                        cloud_msg.header.frame_id, aruco_header.stamp,
                        out_x, out_y, out_range, out_bearing);
                }
                if (index >= aruco_landmark_map_x.size() ||
                    index >= aruco_landmark_map_y.size()) {
                    return false;
                }
                const double map_x = aruco_landmark_map_x[index];
                const double map_y = aruco_landmark_map_y[index];
                if (is_invalid_landmark_xy(map_x, map_y)) {
                    return false;
                }
                return expected_landmark_in_cloud_frame(
                    map_x, map_y, cloud_msg.header.frame_id, cloud_msg.header.stamp,
                    out_x, out_y, out_range, out_bearing);
            };

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::MarkerArray centres;
        visualization_msgs::msg::MarkerArray candidate_markers;
        if (publish_candidate_plane_markers_) {
            visualization_msgs::msg::Marker clear_all;
            clear_all.header = cloud_msg.header;
            clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
            candidate_markers.markers.push_back(clear_all);
        }
        ros2_aruco_interfaces::msg::ArucoMarkers cube_msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr run_plane_inliers(
            new pcl::PointCloud<pcl::PointXYZ>());
        size_t run_tags_preplane_skip = 0;
        size_t run_tags_with_center = 0;
        size_t run_total_planes = 0;
        size_t run_total_side_planes = 0;
        size_t run_total_top_planes = 0;

        for (size_t aruco_idx = 0; aruco_idx < association_count; ++aruco_idx) {
            const auto &aruco_pose = aruco_poses[aruco_idx];
            const float aruco_x = static_cast<float>(aruco_pose.position.x);
            const float aruco_y = static_cast<float>(aruco_pose.position.y);

            double expected_x_cloud = 0.0;
            double expected_y_cloud = 0.0;
            double expected_range_xy = 0.0;
            double expected_bearing_cloud_deg = 0.0;
            const bool have_expected_landmark = expected_position_for_index(
                aruco_idx, expected_x_cloud, expected_y_cloud,
                expected_range_xy, expected_bearing_cloud_deg);

            const double range_for_inlier_heuristic =
                have_expected_landmark
                    ? expected_range_xy
                    : std::max(0.5, std::hypot(
                          static_cast<double>(aruco_x),
                          static_cast<double>(aruco_y)));
            const int dynamic_min_inliers =
                static_cast<int>(0.3 / (range_for_inlier_heuristic * 0.002967) / 3);
            const int min_inliers = std::max(min_inliers_, dynamic_min_inliers);
            const double radial_tol = use_camera_mode
                ? std::max(
                    camera_cone_depth_tolerance_m_,
                    camera_cone_depth_tolerance_ratio_ * expected_range_xy)
                : max_distance_from_aruco_;
            const double aruco_angle_deg_base = atan2(aruco_y, aruco_x) * 180.0 / M_PI;
            const double ref_angle = have_expected_landmark
                                         ? wrap180(expected_bearing_cloud_deg)
                                         : wrap180(aruco_angle_deg_base);

            pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(
                new pcl::PointCloud<pcl::PointXYZ>());
            /* The same neighbourhood without the radial/angular gates. The
             * gates crop a wall down to roughly face size, which is exactly
             * what makes a face-sized patch of one indistinguishable from a
             * cube face; the veto in BoundedPlaneModel needs to see what was
             * cropped away. */
            pcl::PointCloud<pcl::PointXYZ>::Ptr context_cloud(
                new pcl::PointCloud<pcl::PointXYZ>());
            size_t stage_pass_radial = 0;
            size_t stage_pass_both = 0;
            for (size_t point_idx = 0; point_idx < full_cloud_->points.size(); ++point_idx) {
                if (point_marker_ids[point_idx] != aruco_ids[aruco_idx]) {
                    continue;
                }
                const auto &pt = full_cloud_->points[point_idx];
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }
                context_cloud->push_back(pt);
                const double r_point = std::hypot(
                    static_cast<double>(pt.x), static_cast<double>(pt.y));
                if (have_expected_landmark &&
                    std::fabs(expected_range_xy - r_point) >= radial_tol) {
                    continue;
                }
                ++stage_pass_radial;

                const double pt_angle_deg =
                    atan2(static_cast<double>(pt.y), static_cast<double>(pt.x)) *
                    180.0 / M_PI;
                const double active_angular_tolerance = use_camera_mode
                    ? camera_cone_half_angle_deg_ : angular_tolerance_deg_;
                if (angDeltaDeg(pt_angle_deg, ref_angle) < active_angular_tolerance) {
                    ++stage_pass_both;
                    candidate_cloud->push_back(pt);
                }
            }

            /* The veto asks a coarse question -- is there coplanar support in
             * most bearing sectors -- so it needs coverage, not density. Cap
             * the scan cost by striding: it runs inside the RANSAC loop, where
             * the full neighbourhood would dominate the frame budget. */
            if (ransac_context_max_points_ > 0 &&
                context_cloud->size() > static_cast<size_t>(ransac_context_max_points_)) {
                const size_t stride =
                    (context_cloud->size() +
                     static_cast<size_t>(ransac_context_max_points_) - 1) /
                    static_cast<size_t>(ransac_context_max_points_);
                pcl::PointCloud<pcl::PointXYZ>::Ptr thinned(
                    new pcl::PointCloud<pcl::PointXYZ>());
                thinned->reserve(context_cloud->size() / stride + 1);
                for (size_t i = 0; i < context_cloud->size(); i += stride) {
                    thinned->push_back(context_cloud->points[i]);
                }
                context_cloud.swap(thinned);
            }

            if (candidate_cloud->size() < static_cast<size_t>(min_inliers)) {
                ++run_tags_preplane_skip;
                // RCLCPP_WARN_THROTTLE(
                //     this->get_logger(), *this->get_clock(), 2000,
                //     "[detect_cube PLANES] id=%ld skipped before plane fit: "
                //     "candidate=%zu min_inliers=%d pass_radial=%zu pass_angle=%zu",
                //     aruco_ids[aruco_idx], candidate_cloud->size(), min_inliers,
                //     stage_pass_radial, stage_pass_both);
                continue;
            }

            std::vector<PlaneDetection> planes;
            std::vector<PlaneDetection> extracted_planes;
            pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(
                new pcl::PointCloud<pcl::PointXYZ>());
            *remaining = *candidate_cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr all_plane_inliers(
                new pcl::PointCloud<pcl::PointXYZ>());

            for (int plane_idx = 0; plane_idx < max_planes_; ++plane_idx) {
                PlaneDetection plane;
                if (!extract_plane(remaining, context_cloud, min_inliers, plane)) {
                    break;
                }
                extracted_planes.push_back(plane);

                if (publish_candidate_plane_markers_) {
                    append_candidate_plane_markers(
                        plane, aruco_ids[aruco_idx], plane_idx,
                        cloud_msg.header, candidate_markers);
                }

                if (plane.is_side || plane.is_top) {
                    *all_plane_inliers += *plane.inliers;
                    planes.push_back(plane);
                }

                if (!plane.indices ||
                    plane.indices->indices.size() < static_cast<size_t>(min_inliers)) {
                    break;
                }
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(remaining);
                extract.setIndices(plane.indices);
                extract.setNegative(true);
                pcl::PointCloud<pcl::PointXYZ>::Ptr next_remaining(
                    new pcl::PointCloud<pcl::PointXYZ>());
                extract.filter(*next_remaining);
                remaining.swap(next_remaining);
                if (remaining->size() < static_cast<size_t>(min_inliers)) {
                    break;
                }
            }

            if (planes.empty() && accept_best_plane_fallback_ && !extracted_planes.empty()) {
                auto best_it = std::max_element(
                    extracted_planes.begin(),
                    extracted_planes.end(),
                    [](const PlaneDetection &a, const PlaneDetection &b) {
                        return a.inlier_count < b.inlier_count;
                    });
                if (best_it != extracted_planes.end() &&
                    best_it->inlier_count >= static_cast<size_t>(min_inliers) &&
                    (max_face_diagonal_multiplier_ <= 0.0 ||
                     best_it->dim_long <= max_face_diagonal_multiplier_ *
                         std::hypot(cube_width_m_, cube_height_m_))) {
                    PlaneDetection fallback = *best_it;
                    fallback.is_side = true;
                    fallback.is_top = false;
                    *all_plane_inliers += *fallback.inliers;
                    planes.push_back(fallback);
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "[detect_cube PLANES] id=%ld accepting fallback plane: "
                        "inliers=%zu dim_short=%.3f dim_long=%.3f side_score=%.3f top_score=%.3f",
                        aruco_ids[aruco_idx], fallback.inlier_count,
                        fallback.dim_short, fallback.dim_long,
                        fallback.side_score, fallback.top_score);
                }
            }

            if (all_plane_inliers->empty()) {
                ++run_tags_preplane_skip;
                // RCLCPP_WARN_THROTTLE(
                //     this->get_logger(), *this->get_clock(), 2000,
                //     "[detect_cube PLANES] id=%ld no plausible cuboid planes from %zu candidates",
                //     aruco_ids[aruco_idx], candidate_cloud->size());
                continue;
            }

            std::vector<PlaneDetection> side_candidates;
            std::vector<PlaneDetection> top_candidates;
            for (const auto &plane : planes) {
                if (plane.is_side) {
                    side_candidates.push_back(plane);
                }
                if (plane.is_top) {
                    top_candidates.push_back(plane);
                }
            }

            std::sort(side_candidates.begin(), side_candidates.end(),
                [](const PlaneDetection &a, const PlaneDetection &b) {
                    if (std::fabs(a.side_score - b.side_score) > 1e-6) {
                        return a.side_score < b.side_score;
                    }
                    return a.inlier_count > b.inlier_count;
                });
            std::sort(top_candidates.begin(), top_candidates.end(),
                [](const PlaneDetection &a, const PlaneDetection &b) {
                    if (std::fabs(a.top_score - b.top_score) > 1e-6) {
                        return a.top_score < b.top_score;
                    }
                    return a.inlier_count > b.inlier_count;
                });

            if (side_candidates.empty()) {
                ++run_tags_preplane_skip;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "[detect_cube PLANES] id=%ld rejected: no side face candidate",
                    aruco_ids[aruco_idx]);
                continue;
            }

            std::vector<PlaneDetection> side_faces;
            side_faces.push_back(side_candidates.front());
            for (size_t i = 1; i < side_candidates.size() && side_faces.size() < 2; ++i) {
                const double centroid_dist = norm_vec(sub_vec(
                    side_candidates[i].centroid, side_faces.front().centroid));
                const double normal_dot = std::fabs(dot_vec(
                    side_candidates[i].normal, side_faces.front().normal));
                if (centroid_dist <= plane_group_centroid_max_m_ &&
                    normal_dot <= side_perpendicular_dot_max_) {
                    side_faces.push_back(side_candidates[i]);
                }
            }

            bool have_top = false;
            PlaneDetection top_face;
            if (!top_candidates.empty()) {
                for (const auto &candidate : top_candidates) {
                    bool near_side = true;
                    for (const auto &side : side_faces) {
                        const double centroid_dist = norm_vec(sub_vec(
                            candidate.centroid, side.centroid));
                        if (centroid_dist > plane_group_centroid_max_m_) {
                            near_side = false;
                            break;
                        }
                    }
                    if (near_side) {
                        top_face = candidate;
                        have_top = true;
                        break;
                    }
                }
            }

            Vec3 center;
            if (side_faces.size() >= 2u) {
                Vec3 bbox_min{
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()
                };
                Vec3 bbox_max{
                    -std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()
                };
                size_t bbox_point_count = 0;
                auto accumulate_bbox = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
                    if (!cloud) {
                        return;
                    }
                    for (const auto &pt : cloud->points) {
                        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                            continue;
                        }
                        bbox_min.x = std::min(bbox_min.x, static_cast<double>(pt.x));
                        bbox_min.y = std::min(bbox_min.y, static_cast<double>(pt.y));
                        bbox_min.z = std::min(bbox_min.z, static_cast<double>(pt.z));
                        bbox_max.x = std::max(bbox_max.x, static_cast<double>(pt.x));
                        bbox_max.y = std::max(bbox_max.y, static_cast<double>(pt.y));
                        bbox_max.z = std::max(bbox_max.z, static_cast<double>(pt.z));
                        ++bbox_point_count;
                    }
                };

                for (const auto &side : side_faces) {
                    accumulate_bbox(side.inliers);
                }

                if (bbox_point_count == 0) {
                    ++run_tags_preplane_skip;
                    continue;
                }

                // Extend bbox along each face's outward normal by cube_width
                // so that the bbox center lands at the true cube center
                // instead of the visible corner.
                for (const auto &side : side_faces) {
                    Vec3 outward = side.normal;
                    const Vec3 rover_to_face{
                        side.centroid.x, side.centroid.y, side.centroid.z};
                    if (dot_vec(rover_to_face, outward) < 0.0)
                        outward = scale_vec(outward, -1.0);
                    // outward now points away from rover (towards hidden side)
                    if (outward.x > 0.0)
                        bbox_max.x = std::max(bbox_max.x, bbox_max.x + outward.x * cube_width_m_);
                    else
                        bbox_min.x = std::min(bbox_min.x, bbox_min.x + outward.x * cube_width_m_);
                    if (outward.y > 0.0)
                        bbox_max.y = std::max(bbox_max.y, bbox_max.y + outward.y * cube_width_m_);
                    else
                        bbox_min.y = std::min(bbox_min.y, bbox_min.y + outward.y * cube_width_m_);
                }

                center = {
                    0.5 * (bbox_min.x + bbox_max.x),
                    0.5 * (bbox_min.y + bbox_max.y),
                    0.5 * (bbox_min.z + bbox_max.z)
                };
            } else {
                const PlaneDetection &side = side_faces.front();
                Vec3 outward_from_rover = side.normal;
                const Vec3 rover_to_face{
                    side.centroid.x,
                    side.centroid.y,
                    side.centroid.z
                };
                if (dot_vec(rover_to_face, outward_from_rover) < 0.0) {
                    outward_from_rover = scale_vec(outward_from_rover, -1.0);
                }
                center = add_vec(
                    side.centroid, scale_vec(outward_from_rover, cube_width_m_ * 0.5));
            }

            if (have_expected_landmark) {
                const double center_expected_dist = std::hypot(
                    center.x - expected_x_cloud,
                    center.y - expected_y_cloud);
                bool center_outside_gate = false;
                double reported_error = center_expected_dist;
                double reported_limit = max_center_error_m_;
                if (use_camera_mode) {
                    const double center_range = std::hypot(center.x, center.y);
                    const double center_bearing = std::atan2(center.y, center.x);
                    const double expected_bearing = std::atan2(
                        expected_y_cloud, expected_x_cloud);
                    const double bearing_error = std::fabs(wrap180(
                        (center_bearing - expected_bearing) * 180.0 / M_PI));
                    const double radial_error = std::fabs(
                        center_range - expected_range_xy);
                    const double depth_tolerance = std::max(
                        camera_cone_depth_tolerance_m_,
                        camera_cone_depth_tolerance_ratio_ * expected_range_xy);
                    center_outside_gate =
                        radial_error > depth_tolerance ||
                        bearing_error > camera_cone_half_angle_deg_;
                    reported_error = std::max(
                        radial_error / depth_tolerance,
                        bearing_error / camera_cone_half_angle_deg_);
                    reported_limit = 1.0;
                } else {
                    const double max_center_error = max_center_error_m_ > 0.0
                        ? max_center_error_m_
                        : std::max(0.45, max_distance_from_aruco_ + cube_width_m_);
                    center_outside_gate = center_expected_dist > max_center_error;
                    reported_limit = max_center_error;
                }
                if (center_outside_gate) {
                    ++run_tags_preplane_skip;
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "[detect_cube PLANES] id=%ld rejected: center gate error %.3f "
                        "(limit %.3f, mode=%s)",
                        aruco_ids[aruco_idx], reported_error, reported_limit,
                        use_camera_mode ? "camera-cone" : "map-radius");
                    continue;
                }

                bool ambiguous_id = false;
                for (size_t other_idx = 0; other_idx < association_count; ++other_idx) {
                    if (other_idx == aruco_idx ||
                        aruco_ids[other_idx] == aruco_ids[aruco_idx]) {
                        continue;
                    }
                    double other_x = 0.0;
                    double other_y = 0.0;
                    double other_range = 0.0;
                    double other_bearing = 0.0;
                    if (!expected_position_for_index(
                            other_idx, other_x, other_y,
                            other_range, other_bearing)) {
                        continue;
                    }
                    const double other_dist = std::hypot(
                        center.x - other_x, center.y - other_y);
                    if (other_dist <=
                        center_expected_dist + association_ambiguity_margin_m_) {
                        ambiguous_id = true;
                        break;
                    }
                }
                if (ambiguous_id) {
                    ++run_tags_preplane_skip;
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "[detect_cube PLANES] id=%ld rejected: center is not uniquely "
                        "closest to its associated ArUco",
                        aruco_ids[aruco_idx]);
                    continue;
                }
            }

            for (const auto &side : side_faces) {
                *run_plane_inliers += *side.inliers;
            }

            for (size_t i = 0; i < side_faces.size(); ++i) {
                append_plane_markers(
                    side_faces[i],
                    static_cast<int>(aruco_idx * 10 + i),
                    cloud_msg.header,
                    markers);
            }
            if (have_top) {
                append_plane_markers(
                    top_face,
                    static_cast<int>(aruco_idx * 10 + 5),
                    cloud_msg.header,
                    markers);
            }

            geometry_msgs::msg::Point center_point = to_point_msg(center);
            std::string target_frame = cloud_msg.header.frame_id;
            const bool transformed_to_base =
                transform_point_to_base(cloud_msg, center_point, target_frame);

            visualization_msgs::msg::Marker center_marker;
            center_marker.header.stamp = cloud_msg.header.stamp;
            center_marker.header.frame_id = target_frame;
            center_marker.ns = "cube_center_mean";
            center_marker.id = static_cast<int>(aruco_ids[aruco_idx]);
            center_marker.type = visualization_msgs::msg::Marker::SPHERE;
            center_marker.action = visualization_msgs::msg::Marker::ADD;
            center_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
            center_marker.pose.position = center_point;
            center_marker.pose.orientation.w = 1.0;
            center_marker.scale.x = 0.1;
            center_marker.scale.y = 0.1;
            center_marker.scale.z = 0.1;
            center_marker.color.r = 0.0f;
            center_marker.color.g = 1.0f;
            center_marker.color.b = 0.0f;
            center_marker.color.a = 1.0f;
            centres.markers.push_back(center_marker);

            if (!transformed_to_base) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "[detect_cube PLANES] id=%ld center found but base_link transform missing",
                    aruco_ids[aruco_idx]);
                continue;
            }

            cube_msg.marker_ids.push_back(aruco_ids[aruco_idx]);
            geometry_msgs::msg::Pose pose;
            pose.position = center_point;
            pose.orientation.w = 1.0;
            cube_msg.poses.push_back(pose);
            const float angle_deg =
                std::atan2(center_point.y, center_point.x) *
                180.0f / static_cast<float>(M_PI);
            cube_msg.ar_angles_list.push_back(angle_deg);

            ++run_tags_with_center;
            run_total_planes += planes.size();
            run_total_side_planes += side_faces.size();
            run_total_top_planes += have_top ? 1 : 0;

            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "[detect_cube PLANES] id=%ld center_base=(%.3f, %.3f, %.3f) "
            //     "planes=%zu sides=%zu top=%s candidates=%zu min_inliers=%d",
            //     aruco_ids[aruco_idx], center_point.x, center_point.y, center_point.z,
            //     planes.size(), side_faces.size(), have_top ? "yes" : "no",
            //     candidate_cloud->size(), min_inliers);
        }

        markers_pub_->publish(markers);
        centre_pub_->publish(centres);
        if (publish_candidate_plane_markers_) {
            candidate_planes_pub_->publish(candidate_markers);
        }

        sensor_msgs::msg::PointCloud2 cloud_with_planes;
        pcl::toROSMsg(*run_plane_inliers, cloud_with_planes);
        cloud_with_planes.header = cloud_msg.header;
        lines_pub_->publish(cloud_with_planes);

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "[detect_cube PLANES RUN] frame=%s stamp=%d.%09u tags_in_assoc=%zu "
        //     "tags_skipped=%zu tags_with_center=%zu planes=%zu side_planes=%zu top_planes=%zu",
        //     cloud_msg.header.frame_id.c_str(),
        //     cloud_msg.header.stamp.sec,
        //     static_cast<unsigned>(cloud_msg.header.stamp.nanosec),
        //     aruco_ids.size(),
        //     run_tags_preplane_skip,
        //     run_tags_with_center,
        //     run_total_planes,
        //     run_total_side_planes,
        //     run_total_top_planes);

        if (!cube_msg.marker_ids.empty()) {
            cube_msg.header.stamp = cloud_msg.header.stamp;
            cube_msg.header.frame_id = "base_link";
            cube_markers_pub_->publish(cube_msg);
        }
    }

    void detect_lignes(const sensor_msgs::msg::PointCloud2 &cloud_msg) {
        detect_planes(cloud_msg);
    }
    
private:
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string aruco_topic_;
    double distance_threshold_inliers;
    int max_iterations_;
    bool ransac_bounded_plane_enable_{true};
    double ransac_max_plane_short_extent_m_{0.0};
    double ransac_max_plane_long_extent_m_{0.0};
    double ransac_core_radius_margin_{1.0};
    double ransac_max_outside_fraction_{0.35};
    // Resolved from ransac_core_radius_margin_ x the face circumscribed radius.
    double ransac_core_radius_m_{0.0};
    double ransac_bbox_angle_step_deg_{5.0};
    bool ransac_embedded_veto_enable_{true};
    int ransac_embedded_min_bin_points_{3};
    int ransac_embedded_min_occupied_bins_{5};
    int ransac_context_max_points_{600};
    bool ransac_require_vertical_long_axis_{true};
    double ransac_vertical_long_axis_max_deg_{20.0};
    int min_inliers_;
    int max_lines_;
    double t;
    double cube_width_m_;
    double cube_height_m_;
    int max_planes_;
    double plane_group_centroid_max_m_;
    double face_dimension_tolerance_m_;
    double side_perpendicular_dot_max_;
    double top_vertical_dot_min_;
    double face_min_short_frac_;
    double side_min_long_frac_;
    double top_min_long_frac_;
    double face_score_tolerance_multiplier_;
    double max_face_diagonal_multiplier_;
    double max_center_error_m_;
    bool accept_best_plane_fallback_;
    double process_rate_hz_;
    double marker_lifetime_sec_;
    bool publish_candidate_plane_markers_{true};
    double max_distance_from_aruco_;
    double angular_tolerance_deg_;
    std::atomic_bool use_camera_aruco_position_{false};
    double camera_cone_half_angle_deg_{5.0};
    double camera_cone_depth_tolerance_m_{1.0};
    double camera_cone_depth_tolerance_ratio_{0.20};
    double association_ambiguity_margin_m_;
    double merge_duplicate_2d_line_mid_max_m_;
    double merge_duplicate_2d_parallel_min_dir_dot_;
    double opposite_2d_pair_min_dir_dot_;
    double opposite_2d_mid_sep_min_frac_;
    double opposite_2d_mid_sep_max_frac_;
    std::string map_frame_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lines_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;      
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centre_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_planes_pub_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_subscriber_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::vector<int64_t> aruco_ids_;
    std::vector<geometry_msgs::msg::Pose> aruco_poses_;
    std::vector<double> aruco_landmark_map_x_;
    std::vector<double> aruco_landmark_map_y_;
    std_msgs::msg::Header aruco_header_;
    std::mutex aruco_mutex_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr cube_markers_pub_;

    int64_t min_process_period_ns_;
    int64_t last_process_time_ns_;
    std::atomic<int64_t> mode_switch_guard_until_ns_{0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCubeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
