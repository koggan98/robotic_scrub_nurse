#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tracking_pkg/msg/grasp_candidate.hpp>
#include <tracking_pkg/srv/build_world_model.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
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

}  // namespace

class ToolPickTestNode : public rclcpp::Node {
public:
    ToolPickTestNode()
    : Node("tool_pick_test_node") {
        tool_z_m_ = declare_parameter("tool_z_m", 0.045);
        approach_height_m_ = declare_parameter("approach_height_m", 0.05);
        tool_yaw_offset_rad_ = declare_parameter("tool_yaw_offset_rad", 1.57079632679);
        move_group_name_ = declare_parameter("move_group_name", std::string("ur_manipulator"));
        end_effector_link_ = declare_parameter("end_effector_link", std::string("gripper_tip_link"));
        reference_frame_ = declare_parameter("reference_frame", std::string("world"));
        velocity_scale_ = declare_parameter("velocity_scale", 0.2);
        acceleration_scale_ = declare_parameter("acceleration_scale", 0.2);
        gripper_pause_seconds_ = declare_parameter("gripper_pause_seconds", 1.0);

        build_world_model_client_ =
            create_client<tracking_pkg::srv::BuildWorldModel>("/build_world_model");
        gripper_mover_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);

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

        return executePick(candidate);
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
                << " xy=(" << p.x << ", " << p.y << ")\n";
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
        grasp_pose.position.z = tool_z_m_;
        grasp_pose.orientation = topDownQuaternionFromHandleAxis(
            candidate.handle_axis,
            tool_yaw_offset_rad_);

        geometry_msgs::msg::Pose approach_pose = grasp_pose;
        approach_pose.position.z = tool_z_m_ + approach_height_m_;

        RCLCPP_INFO(
            get_logger(),
            "Selected %s (%s). Grasp xyz=(%.4f, %.4f, %.4f), approach z=%.4f, yaw_offset=%.4f rad.",
            candidate.tool_id.c_str(),
            candidate.tool_class.c_str(),
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

        if (!moveLinearToPose(grasp_pose, "Descended linearly to fixed tool height.")) {
            return false;
        }

        publishGripper(false);
        sleepForGripper();

        if (!moveLinearToPose(approach_pose, "Lifted linearly back above selected tool.")) {
            return false;
        }

        RCLCPP_INFO(get_logger(), "Pick-test sequence complete.");
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

    void publishGripper(const bool open) {
        auto msg = std_msgs::msg::Bool();
        msg.data = open;
        gripper_mover_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "%s gripper command published.", open ? "Open" : "Close");
    }

    void sleepForGripper() {
        const auto duration = std::chrono::duration<double>(std::max(0.0, gripper_pause_seconds_));
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }

    double tool_z_m_;
    double approach_height_m_;
    double tool_yaw_offset_rad_;
    std::string move_group_name_;
    std::string end_effector_link_;
    std::string reference_frame_;
    double velocity_scale_;
    double acceleration_scale_;
    double gripper_pause_seconds_;

    rclcpp::Client<BuildWorldModel>::SharedPtr build_world_model_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToolPickTestNode>();
    const bool ok = node->run();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
