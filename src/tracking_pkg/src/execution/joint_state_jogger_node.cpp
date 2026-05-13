#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

std::vector<double> declareVector(
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

void printMenu() {
    std::cout
        << "\nJoint-State Jogger\n"
        << "==================\n"
        << "  1 - tray_left\n"
        << "  2 - tray_center\n"
        << "  3 - tray_right\n"
        << "  4 - reclaim_holder\n"
        << "  q - quit\n"
        << "Select target: " << std::flush;
}

}  // namespace

class JointStateJoggerNode : public rclcpp::Node {
public:
    JointStateJoggerNode()
    : Node("joint_state_jogger_node") {
        move_group_name_ = declare_parameter("move_group_name", std::string("ur_manipulator"));
        velocity_scale_ = declare_parameter("velocity_scale", 0.6);
        acceleration_scale_ = declare_parameter("acceleration_scale", 0.6);
        planning_time_ = declare_parameter("planning_time", 2.0);

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

        waypoints_["tray_left"] = declareVector(
            *this,
            "joint_state.tray_left",
            {-0.1601136366, -2.2975937329, 2.2748802344, -1.5248240244, -1.2305892150, 0.0},
            6);
        waypoints_["tray_center"] = declareVector(
            *this,
            "joint_state.tray_center",
            {1.6273860335, -2.6157323323, 2.2653740088, -1.5361764350, -1.5710740725, 0.0},
            6);
        waypoints_["tray_right"] = declareVector(
            *this,
            "joint_state.tray_right",
            {2.1262052059, -2.1099254094, 2.4224575202, -2.1711937390, -1.6874073187, 0.0},
            6);
        waypoints_["reclaim_holder"] = declareVector(
            *this,
            "joint_state.reclaim_holder",
            {-1.3372991721, -1.6360527478, 2.1957047621, -2.1333886586, -1.5137208144, 0.0},
            6);
        menu_targets_ = {
            {"1", "tray_left"},
            {"2", "tray_center"},
            {"3", "tray_right"},
            {"4", "reclaim_holder"},
        };
    }

    void run() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), move_group_name_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        RCLCPP_INFO(
            get_logger(),
            "Joint-state jogger ready (move_group=%s, velocity=%.2f, acceleration=%.2f).",
            move_group_name_.c_str(),
            velocity_scale_,
            acceleration_scale_);

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            printMenu();

            std::string input;
            if (!std::getline(std::cin, input)) {
                RCLCPP_ERROR(get_logger(), "Failed to read terminal input.");
                break;
            }
            input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());

            if (input == "q" || input == "Q") {
                RCLCPP_INFO(get_logger(), "Joint-state jogger stopped by user.");
                break;
            }

            const auto target_it = menu_targets_.find(input);
            if (target_it == menu_targets_.end()) {
                RCLCPP_WARN(get_logger(), "Unknown selection '%s'.", input.c_str());
                continue;
            }

            moveToTarget(target_it->second);
        }
    }

private:
    bool moveToTarget(const std::string &target_name) {
        if (isCrossTrayMove(target_name)) {
            RCLCPP_INFO(
                get_logger(),
                "Routing cross-tray waypoint move via tray_center before '%s'.",
                target_name.c_str());
            if (!moveToWaypoint("tray_center")) {
                return false;
            }
        }

        return moveToWaypoint(target_name);
    }

    bool isCrossTrayMove(const std::string &target_name) const {
        return (target_name == "tray_left" && isAtOrLastReached("tray_right"))
               || (target_name == "tray_right" && isAtOrLastReached("tray_left"));
    }

    bool isAtOrLastReached(const std::string &waypoint_name) const {
        if (last_reached_waypoint_ == waypoint_name) {
            return true;
        }

        const auto waypoint_it = waypoints_.find(waypoint_name);
        if (waypoint_it == waypoints_.end() || !move_group_) {
            return false;
        }

        const auto current_positions = move_group_->getCurrentJointValues();
        const auto &target_positions = waypoint_it->second;
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

    bool moveToWaypoint(const std::string &name) {
        const auto waypoint_it = waypoints_.find(name);
        if (waypoint_it == waypoints_.end()) {
            RCLCPP_ERROR(get_logger(), "Waypoint '%s' is not configured.", name.c_str());
            return false;
        }

        const auto &positions = waypoint_it->second;
        if (positions.size() != joint_state_names_.size()) {
            RCLCPP_ERROR(
                get_logger(),
                "Waypoint '%s' has %zu positions but %zu joint names are configured.",
                name.c_str(),
                positions.size(),
                joint_state_names_.size());
            return false;
        }

        std::map<std::string, double> joint_target;
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            joint_target[joint_state_names_[i]] = positions[i];
        }

        RCLCPP_INFO(get_logger(), "Planning to waypoint '%s'.", name.c_str());
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            RCLCPP_INFO(
                get_logger(),
                "  %s = %.6f",
                joint_state_names_[i].c_str(),
                positions[i]);
        }

        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(planning_time_);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Planning to waypoint '%s' failed.", name.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Executing waypoint '%s'.", name.c_str());
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Execution to waypoint '%s' failed.", name.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Reached waypoint '%s'.", name.c_str());
        last_reached_waypoint_ = name;
        return true;
    }

    std::string move_group_name_;
    double velocity_scale_;
    double acceleration_scale_;
    double planning_time_;
    std::vector<std::string> joint_state_names_;
    std::map<std::string, std::vector<double>> waypoints_;
    std::map<std::string, std::string> menu_targets_;
    std::string last_reached_waypoint_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateJoggerNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
