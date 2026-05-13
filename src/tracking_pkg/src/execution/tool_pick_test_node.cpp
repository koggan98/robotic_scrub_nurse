#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tracking_pkg/msg/grasp_candidate.hpp>
#include <tracking_pkg/msg/tool_detection_array.hpp>
#include <tracking_pkg/srv/build_world_model.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

struct Vec3 {
    double x;
    double y;
    double z;
};

double norm(const Vec3 &v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalizeOrDefault(Vec3 v, const Vec3 &fallback) {
    const double n = norm(v);
    if (n < 1e-6) {
        return fallback;
    }
    return Vec3{v.x / n, v.y / n, v.z / n};
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

geometry_msgs::msg::Quaternion quaternionFromRotationMatrix(
    const double r00, const double r01, const double r02,
    const double r10, const double r11, const double r12,
    const double r20, const double r21, const double r22) {
    geometry_msgs::msg::Quaternion q;
    const double trace = r00 + r11 + r22;

    if (trace > 0.0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (r21 - r12) * s;
        q.y = (r02 - r20) * s;
        q.z = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
        q.w = (r21 - r12) / s;
        q.x = 0.25 * s;
        q.y = (r01 + r10) / s;
        q.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
        q.w = (r02 - r20) / s;
        q.x = (r01 + r10) / s;
        q.y = 0.25 * s;
        q.z = (r12 + r21) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
        q.w = (r10 - r01) / s;
        q.x = (r02 + r20) / s;
        q.y = (r12 + r21) / s;
        q.z = 0.25 * s;
    }

    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (q_norm > 1e-9) {
        q.x /= q_norm;
        q.y /= q_norm;
        q.z /= q_norm;
        q.w /= q_norm;
    }
    return q;
}

geometry_msgs::msg::Quaternion topDownQuaternionFromHandleAxis(
    const geometry_msgs::msg::Vector3 &handle_axis,
    const double yaw_offset_rad) {
    Vec3 target_x{handle_axis.x, handle_axis.y, 0.0};
    target_x = normalizeOrDefault(target_x, Vec3{1.0, 0.0, 0.0});

    const double c = std::cos(yaw_offset_rad);
    const double s = std::sin(yaw_offset_rad);
    target_x = normalizeOrDefault(
        Vec3{
            c * target_x.x - s * target_x.y,
            s * target_x.x + c * target_x.y,
            0.0,
        },
        Vec3{0.0, 1.0, 0.0});

    const Vec3 target_z{0.0, 0.0, -1.0};
    Vec3 target_y = normalizeOrDefault(cross(target_z, target_x), Vec3{0.0, -1.0, 0.0});

    return quaternionFromRotationMatrix(
        target_x.x, target_y.x, target_z.x,
        target_x.y, target_y.y, target_z.y,
        target_x.z, target_y.z, target_z.z);
}

bool isFinitePoint(const geometry_msgs::msg::Point &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool hasUsableWorldXy(const tracking_pkg::msg::GraspCandidate &candidate) {
    const auto &point = candidate.grasp_pose.pose.position;
    if (!isFinitePoint(point)) {
        return false;
    }
    if (candidate.grasp_pose.header.frame_id != "world") {
        return false;
    }
    return std::hypot(point.x, point.y) > 1e-4;
}

std::vector<double> parameterVectorOrDefault(
    rclcpp::Node &node,
    const std::string &name,
    const std::vector<double> &fallback,
    const size_t expected_size) {
    const auto value = node.declare_parameter(name, fallback);
    if (value.size() == expected_size) {
        return value;
    }
    RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' has %zu values, expected %zu. Using default.",
        name.c_str(),
        value.size(),
        expected_size);
    return fallback;
}

}  // namespace

class ToolPickTestNode : public rclcpp::Node {
public:
    ToolPickTestNode()
    : Node("tool_pick_test_node") {
        z_offset_m_ = declare_parameter("z_offset", -0.000);
        approach_height_m_ = declare_parameter("approach_height_m", 0.05);
        tool_yaw_offset_rad_ = declare_parameter("tool_yaw_offset_rad", 1.57079632679);
        move_group_name_ = declare_parameter("move_group_name", std::string("ur_manipulator"));
        end_effector_link_ = declare_parameter("end_effector_link", std::string("gripper_tip_link"));
        reference_frame_ = declare_parameter("reference_frame", std::string("world"));
        velocity_scale_ = declare_parameter("velocity_scale", 0.6);
        acceleration_scale_ = declare_parameter("acceleration_scale", 0.6);
        gripper_pause_seconds_ = declare_parameter("gripper_pause_seconds", 1.0);
        handover_planning_time_ = declare_parameter("handover_planning_time", 1.0);
        handover_velocity_scale_ = declare_parameter("handover_velocity_scale", 0.6);
        handover_acceleration_scale_ = declare_parameter("handover_acceleration_scale", 0.6);
        pre_release_dwell_seconds_ = declare_parameter("pre_release_dwell_seconds", 0.3);
        post_zeroer_settle_seconds_ = declare_parameter("post_zeroer_settle_seconds", 0.0);
        post_open_pause_seconds_ = declare_parameter("post_open_pause_seconds", 1.0);
        return_home_after_handover_ = declare_parameter("return_home_after_handover", true);
        tray_image_width_px_ = declare_parameter("tray_image_width_px", 1280);
        enable_region_preposition_ = declare_parameter("enable_region_preposition", true);
        enable_post_pick_waypoints_ = declare_parameter("enable_post_pick_waypoints", true);

        joint_state_names_ = declare_parameter(
            "joint_state_names",
            std::vector<std::string>{
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            });
        if (joint_state_names_.size() != 6) {
            RCLCPP_WARN(
                get_logger(),
                "Parameter 'joint_state_names' has %zu values, expected 6. Using canonical UR order.",
                joint_state_names_.size());
            joint_state_names_ = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            };
        }

        const auto hand_offset = parameterVectorOrDefault(
            *this, "hand_offset", {-0.08, 0.0, 0.05}, 3);
        hand_offset_.x = hand_offset[0];
        hand_offset_.y = hand_offset[1];
        hand_offset_.z = hand_offset[2];

        const auto handover_orientation = parameterVectorOrDefault(
            *this, "handover_orientation", {-0.63, 0.63, -0.321, 0.321}, 4);
        handover_orientation_.x = handover_orientation[0];
        handover_orientation_.y = handover_orientation[1];
        handover_orientation_.z = handover_orientation[2];
        handover_orientation_.w = handover_orientation[3];

        home_joints_ = parameterVectorOrDefault(
            *this,
            "home_joints",
            {-0.1601136366, -2.2975937329, 2.2748802344, -1.5248240244, -1.2305892150, -4.8166621367},
            6);
        joint_state_waypoints_["tray_left"] = parameterVectorOrDefault(
            *this,
            "joint_state.tray_left",
            {-0.1601136366, -2.2975937329, 2.2748802344, -1.5248240244, -1.2305892150, -4.8166621367},
            6);
        joint_state_waypoints_["tray_center"] = parameterVectorOrDefault(
            *this,
            "joint_state.tray_center",
            {1.6273860335, -2.6157323323, 2.2653740088, -1.5361764350, -1.5710740725, -3.1055713336},
            6);
        joint_state_waypoints_["tray_right"] = parameterVectorOrDefault(
            *this,
            "joint_state.tray_right",
            {2.1262052059, -2.1099254094, 2.4224575202, -2.1711937390, -1.6874073187, -2.6279209296},
            6);
        joint_state_waypoints_["reclaim_holder"] = parameterVectorOrDefault(
            *this,
            "joint_state.reclaim_holder",
            {-1.3372991721, -1.6360527478, 2.1957047621, -2.1333886586, -1.5137208144, -6.0057201723},
            6);
        build_world_model_client_ =
            create_client<tracking_pkg::srv::BuildWorldModel>("/build_world_model");
        gripper_mover_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);
        gripper_zeroer_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_zeroer", 10);
        hand_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/hand_pose",
            10,
            std::bind(&ToolPickTestNode::handPoseCallback, this, std::placeholders::_1));
        gripper_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/gripper_done",
            10,
            std::bind(&ToolPickTestNode::gripperDoneCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Tool pick test node ready.");
    }

    bool run() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), move_group_name_);
        move_group_->setEndEffectorLink(end_effector_link_);
        move_group_->setPoseReferenceFrame(reference_frame_);
        move_group_->setPlanningTime(2.0);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        auto response = requestWorldModel();
        if (!response) {
            return false;
        }
        if (!response->success) {
            RCLCPP_ERROR(get_logger(), "World model build failed: %s", response->message.c_str());
            return false;
        }
        if (response->world_model_json.find("\"world_frame_available\": true") == std::string::npos) {
            RCLCPP_ERROR(
                get_logger(),
                "World model has no world-frame TF. Refusing to move with invalid grasp coordinates.");
            return false;
        }

        std::vector<tracking_pkg::msg::GraspCandidate> candidates;
        for (const auto &candidate : response->grasp_candidates.candidates) {
            if (hasUsableWorldXy(candidate)) {
                candidates.push_back(candidate);
            } else {
                RCLCPP_WARN(
                    get_logger(),
                    "Ignoring invalid candidate '%s' (%s): frame='%s', xy=(%.4f, %.4f).",
                    candidate.tool_id.c_str(),
                    candidate.tool_class.c_str(),
                    candidate.grasp_pose.header.frame_id.c_str(),
                    candidate.grasp_pose.pose.position.x,
                    candidate.grasp_pose.pose.position.y);
            }
        }

        if (candidates.empty()) {
            RCLCPP_WARN(get_logger(), "No grasp candidates returned by /build_world_model.");
            RCLCPP_WARN(
                get_logger(),
                "This usually means depth projection failed at the grasp pixel or the "
                "world -> tray_camera_color_optical_frame TF is wrong/missing.");
            return false;
        }

        printCandidates(candidates);
        const int selected_index = readSelection(static_cast<int>(candidates.size()));
        if (selected_index < 0) {
            return false;
        }

        const auto &candidate = candidates[static_cast<size_t>(selected_index)];
        if (!isFinitePoint(candidate.grasp_pose.pose.position)) {
            RCLCPP_ERROR(get_logger(), "Selected candidate has invalid grasp coordinates.");
            return false;
        }

        const std::string tray_region = determineTrayRegion(candidate, response->tools_detected);
        if (enable_region_preposition_) {
            if (!moveToNamedJointStateRouted(
                    tray_region,
                    "Reached region preposition '" + tray_region + "' before camera pick.",
                    "Planning/execution to region preposition '" + tray_region + "' failed.")) {
                return false;
            }
        }

        if (!executePick(candidate)) {
            return false;
        }

        if (enable_post_pick_waypoints_) {
            if (!moveToNamedJointStateRouted(
                    "tray_left",
                    "Reached tray_left post-pick waypoint.",
                    "Planning/execution to tray_left post-pick waypoint failed.")) {
                return false;
            }
        }

        tool_picked_up_ = true;
        RCLCPP_INFO(
            get_logger(),
            "Tool picked up. Waiting for /hand_pose gesture before force-guided handover.");
        waitForHandoverCompletion();
        return !handover_failed_ && !tool_picked_up_ && !waiting_for_gripper_done_ && !handover_in_progress_;
    }

private:
    using BuildWorldModel = tracking_pkg::srv::BuildWorldModel;

    BuildWorldModel::Response::SharedPtr requestWorldModel() {
        RCLCPP_INFO(get_logger(), "Waiting for /build_world_model service...");
        while (rclcpp::ok()
               && !build_world_model_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Still waiting for /build_world_model...");
        }
        if (!rclcpp::ok()) {
            return nullptr;
        }

        RCLCPP_INFO(get_logger(), "Calling /build_world_model once.");
        auto request = std::make_shared<BuildWorldModel::Request>();
        auto future = build_world_model_client_->async_send_request(request);
        const auto result =
            rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(20));
        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Timed out or interrupted while waiting for /build_world_model.");
            return nullptr;
        }
        return future.get();
    }

    void printCandidates(const std::vector<tracking_pkg::msg::GraspCandidate> &candidates) {
        std::cout << "\nDetected grasp candidates:\n";
        for (size_t i = 0; i < candidates.size(); ++i) {
            const auto &candidate = candidates[i];
            const auto &p = candidate.grasp_pose.pose.position;
            std::cout
                << "  [" << i << "] "
                << "tool_id=" << candidate.tool_id
                << " class=" << candidate.tool_class
                << " confidence=" << candidate.grasp_confidence
                << " xyz=(" << p.x << ", " << p.y << ", " << p.z << ")\n";
        }
        std::cout << std::endl;
    }

    int readSelection(const int candidate_count) {
        std::cout << "Select tool index to pick, or 'q' to abort: " << std::flush;
        std::string input;
        if (!std::getline(std::cin, input)) {
            RCLCPP_ERROR(get_logger(), "Failed to read terminal selection.");
            return -1;
        }
        if (input == "q" || input == "Q") {
            RCLCPP_INFO(get_logger(), "Pick test aborted by user.");
            return -1;
        }

        try {
            const int index = std::stoi(input);
            if (index < 0 || index >= candidate_count) {
                RCLCPP_ERROR(get_logger(), "Selection %d is outside the valid range.", index);
                return -1;
            }
            return index;
        } catch (const std::exception &) {
            RCLCPP_ERROR(get_logger(), "Invalid selection '%s'.", input.c_str());
            return -1;
        }
    }

    bool executePick(const tracking_pkg::msg::GraspCandidate &candidate) {
        geometry_msgs::msg::Pose grasp_pose;
        grasp_pose.position.x = candidate.grasp_pose.pose.position.x;
        grasp_pose.position.y = candidate.grasp_pose.pose.position.y;
        const double detected_surface_z = candidate.grasp_pose.pose.position.z;
        grasp_pose.position.z = detected_surface_z + z_offset_m_;
        grasp_pose.orientation = topDownQuaternionFromHandleAxis(
            candidate.handle_axis,
            tool_yaw_offset_rad_);

        geometry_msgs::msg::Pose approach_pose = grasp_pose;
        approach_pose.position.z = grasp_pose.position.z + approach_height_m_;

        RCLCPP_INFO(
            get_logger(),
            "Selected %s (%s). Detected surface z=%.4f, z_offset=%.4f, "
            "final grasp xyz=(%.4f, %.4f, %.4f), approach z=%.4f, yaw_offset=%.4f rad.",
            candidate.tool_id.c_str(),
            candidate.tool_class.c_str(),
            detected_surface_z,
            z_offset_m_,
            grasp_pose.position.x,
            grasp_pose.position.y,
            grasp_pose.position.z,
            approach_pose.position.z,
            tool_yaw_offset_rad_);
        RCLCPP_INFO(
            get_logger(),
            "Target quaternion xyzw=(%.5f, %.5f, %.5f, %.5f).",
            grasp_pose.orientation.x,
            grasp_pose.orientation.y,
            grasp_pose.orientation.z,
            grasp_pose.orientation.w);

        if (!moveToPoseTarget(approach_pose, "Reached approach pose above selected tool.")) {
            return false;
        }

        publishGripper(true);
        sleepForGripper();

        if (!moveLinearToPose(grasp_pose, "Descended linearly to detected tool height.")) {
            return false;
        }

        publishGripper(false);
        sleepForGripper();

        if (!moveLinearToPose(approach_pose, "Lifted linearly back above selected tool.")) {
            return false;
        }

        RCLCPP_INFO(get_logger(), "Pick sequence complete.");
        return true;
    }

    bool moveToPoseTarget(const geometry_msgs::msg::Pose &target_pose, const char *success_log) {
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Planning to approach pose failed.");
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Execution to approach pose failed.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "%s", success_log);
        return true;
    }

    bool moveLinearToPose(const geometry_msgs::msg::Pose &target_pose, const char *success_log) {
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = move_group_->computeCartesianPath(
            waypoints,
            0.005,
            0.0,
            trajectory);
        if (fraction < 0.99) {
            RCLCPP_ERROR(
                get_logger(),
                "Cartesian path planning failed: only %.2f%% of path was planned.",
                fraction * 100.0);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Cartesian path execution failed.");
            return false;
        }

        RCLCPP_INFO(get_logger(), "%s", success_log);
        return true;
    }

    std::string determineTrayRegion(
        const tracking_pkg::msg::GraspCandidate &candidate,
        const tracking_pkg::msg::ToolDetectionArray &tools_detected) {
        double center_x = std::numeric_limits<double>::quiet_NaN();
        bool found_detection = false;
        for (const auto &detection : tools_detected.detections) {
            if (detection.tool_id == candidate.tool_id) {
                center_x = static_cast<double>(detection.body_obb.center_x);
                found_detection = true;
                break;
            }
        }

        if (!found_detection) {
            RCLCPP_WARN(
                get_logger(),
                "Could not find ToolDetection for selected candidate '%s'. Falling back to tray_center.",
                candidate.tool_id.c_str());
            return "tray_center";
        }

        const double image_width = static_cast<double>(std::max(1, tray_image_width_px_));
        const double left_boundary = image_width / 3.0;
        const double right_boundary = 2.0 * image_width / 3.0;

        std::string region = "tray_center";
        if (center_x < left_boundary) {
            region = "tray_left";
        } else if (center_x > right_boundary) {
            region = "tray_right";
        }

        RCLCPP_INFO(
            get_logger(),
            "Selected candidate '%s' has body_obb.center_x=%.1f px in %.0f px image -> %s.",
            candidate.tool_id.c_str(),
            center_x,
            image_width,
            region.c_str());
        return region;
    }

    bool moveToNamedJointStateRouted(
        const std::string &state_name,
        const std::string &success_log,
        const std::string &failure_log) {
        if (isCrossTrayMove(state_name)) {
            RCLCPP_INFO(
                get_logger(),
                "Routing cross-tray waypoint move via tray_center before '%s'.",
                state_name.c_str());
            if (!moveToNamedJointState(
                    "tray_center",
                    "Reached tray_center cross-tray waypoint.",
                    "Planning/execution to tray_center cross-tray waypoint failed.")) {
                return false;
            }
        }

        return moveToNamedJointState(state_name, success_log, failure_log);
    }

    bool isCrossTrayMove(const std::string &target_state_name) const {
        return (target_state_name == "tray_left" && isAtOrLastReached("tray_right"))
               || (target_state_name == "tray_right" && isAtOrLastReached("tray_left"));
    }

    bool isAtOrLastReached(const std::string &state_name) const {
        if (last_reached_joint_state_ == state_name) {
            return true;
        }

        const auto it = joint_state_waypoints_.find(state_name);
        if (it == joint_state_waypoints_.end() || !move_group_) {
            return false;
        }

        const auto current_positions = move_group_->getCurrentJointValues();
        const auto &target_positions = it->second;
        if (current_positions.size() != target_positions.size()) {
            return false;
        }

        constexpr double joint_tolerance_rad = 0.08;
        for (size_t i = 0; i < target_positions.size(); ++i) {
            if (std::abs(current_positions[i] - target_positions[i]) > joint_tolerance_rad) {
                return false;
            }
        }
        return true;
    }

    bool moveToNamedJointState(
        const std::string &state_name,
        const std::string &success_log,
        const std::string &failure_log) {
        const auto it = joint_state_waypoints_.find(state_name);
        if (it == joint_state_waypoints_.end()) {
            RCLCPP_ERROR(get_logger(), "Unknown joint-state waypoint '%s'.", state_name.c_str());
            return false;
        }
        if (!moveToJointPositions(it->second, success_log, failure_log)) {
            return false;
        }
        last_reached_joint_state_ = state_name;
        return true;
    }

    bool moveToJointPositions(
        const std::vector<double> &joint_positions,
        const std::string &success_log,
        const std::string &failure_log) {
        if (joint_state_names_.size() != joint_positions.size()) {
            RCLCPP_ERROR(
                get_logger(),
                "Joint target size mismatch: %zu joint names but %zu positions.",
                joint_state_names_.size(),
                joint_positions.size());
            return false;
        }

        std::map<std::string, double> joint_target;
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            joint_target[joint_state_names_[i]] = joint_positions[i];
        }

        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(2.0);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "%s", failure_log.c_str());
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "%s", failure_log.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "%s", success_log.c_str());
        return true;
    }

    void publishGripper(const bool open) {
        auto msg = std_msgs::msg::Bool();
        msg.data = open;
        gripper_mover_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "%s gripper command published.", open ? "Open" : "Close");
    }

    void publishGripperZeroer(const bool active) {
        auto msg = std_msgs::msg::Bool();
        msg.data = active;
        gripper_zeroer_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Gripper zeroer %s.", active ? "activated" : "deactivated");
    }

    void sleepForGripper() {
        const auto duration = std::chrono::duration<double>(std::max(0.0, gripper_pause_seconds_));
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }

    void waitForHandoverCompletion() {
        rclcpp::Rate rate(20.0);
        while (rclcpp::ok() && (tool_picked_up_ || waiting_for_gripper_done_ || handover_in_progress_)) {
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

    void handPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Hand detected: received /hand_pose message in world frame.");

        if (!tool_picked_up_) {
            RCLCPP_INFO(get_logger(), "Action rejected: hand pose received but no picked tool is active.");
            return;
        }
        if (handover_in_progress_ || waiting_for_gripper_done_) {
            RCLCPP_INFO(get_logger(), "Action rejected: handover or force release is already active.");
            return;
        }

        geometry_msgs::msg::Pose target_pose = *msg;
        target_pose.position.x += hand_offset_.x;
        target_pose.position.y += hand_offset_.y;
        target_pose.position.z += hand_offset_.z;
        target_pose.orientation = handover_orientation_;

        RCLCPP_INFO(
            get_logger(),
            "Action accepted: gesture detected, planning handover to x=%.4f y=%.4f z=%.4f.",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);
        performHandoverToHandPose(target_pose);
    }

    void performHandoverToHandPose(const geometry_msgs::msg::Pose &target_pose) {
        handover_in_progress_ = true;
        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(handover_planning_time_);
        move_group_->setMaxVelocityScalingFactor(handover_velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(handover_acceleration_scale_);
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Reachability decision: unreachable.");
            RCLCPP_ERROR(get_logger(), "Planning to handover pose failed. Keeping tool and waiting for another hand pose.");
            move_group_->clearPoseTargets();
            handover_in_progress_ = false;
            return;
        }

        RCLCPP_INFO(get_logger(), "Reachability decision: reachable.");
        RCLCPP_INFO(get_logger(), "Moving to handover pose.");
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Reachability decision: unreachable.");
            RCLCPP_ERROR(get_logger(), "Execution to handover pose failed. Keeping tool and waiting for another hand pose.");
            move_group_->clearPoseTargets();
            handover_in_progress_ = false;
            return;
        }

        move_group_->clearPoseTargets();
        RCLCPP_INFO(get_logger(), "Reached handover pose, holding before release sensing.");
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, pre_release_dwell_seconds_))));
        RCLCPP_INFO(get_logger(), "Activating gripper sensing after dwell.");
        publishGripperZeroer(true);
        waiting_for_gripper_done_ = true;
        handover_in_progress_ = false;
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_zeroer_settle_seconds_))));
        RCLCPP_INFO(get_logger(), "Waiting for /gripper_done from force-guided release.");
    }

    void gripperDoneCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }
        if (!waiting_for_gripper_done_) {
            RCLCPP_WARN(get_logger(), "Ignoring /gripper_done because no handover release is waiting.");
            return;
        }

        publishGripperZeroer(false);
        waiting_for_gripper_done_ = false;
        tool_picked_up_ = false;

        RCLCPP_INFO(get_logger(), "Gripper opened by force-guided release.");
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_open_pause_seconds_))));

        if (return_home_after_handover_) {
            RCLCPP_INFO(get_logger(), "Returning home after handover release pause.");
            if (!moveToHomePositionUsingJoints()) {
                RCLCPP_ERROR(get_logger(), "Home motion after handover failed.");
                handover_failed_ = true;
                return;
            }
        }

        RCLCPP_INFO(get_logger(), "Force-guided handover complete.");
    }

    bool moveToHomePositionUsingJoints() {
        if (home_joints_.size() != 6) {
            RCLCPP_ERROR(get_logger(), "Cannot move home: home_joints must contain 6 values.");
            return false;
        }
        return moveToJointPositions(
            home_joints_,
            "Reached configured home joint pose.",
            "Planning/execution home joint motion failed.");
    }

    double z_offset_m_;
    double approach_height_m_;
    double tool_yaw_offset_rad_;
    double handover_planning_time_;
    double handover_velocity_scale_;
    double handover_acceleration_scale_;
    double pre_release_dwell_seconds_;
    double post_zeroer_settle_seconds_;
    double post_open_pause_seconds_;
    bool return_home_after_handover_;
    int tray_image_width_px_;
    bool enable_region_preposition_;
    bool enable_post_pick_waypoints_;
    std::string move_group_name_;
    std::string end_effector_link_;
    std::string reference_frame_;
    double velocity_scale_;
    double acceleration_scale_;
    double gripper_pause_seconds_;
    geometry_msgs::msg::Point hand_offset_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    std::vector<std::string> joint_state_names_;
    std::vector<double> home_joints_;
    std::map<std::string, std::vector<double>> joint_state_waypoints_;
    std::string last_reached_joint_state_;
    bool tool_picked_up_ = false;
    bool handover_in_progress_ = false;
    bool waiting_for_gripper_done_ = false;
    bool handover_failed_ = false;

    rclcpp::Client<BuildWorldModel>::SharedPtr build_world_model_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToolPickTestNode>();
    const bool ok = node->run();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
