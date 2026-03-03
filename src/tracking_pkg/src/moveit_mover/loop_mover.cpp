#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// mit ros2 topic pub /tool_selection std_msgs/msg/String "data: '0'" resetten, 1-6 sind werkzeuge

// gripper_mover true = gripper open
// gripper_mover false = gripper close
// gripper_zeroer true = sensing aktiv
// gripper_zeroer false = sensing inaktiv

enum class GrabStrategy {
    Standard,
    HammerCartesian,
    LongScissor,
    RetractorCartesian
};

struct ToolProfile {
    std::string tool_id;
    GrabStrategy strategy = GrabStrategy::Standard;
    geometry_msgs::msg::Point pick_position;
    geometry_msgs::msg::Quaternion pick_orientation;
    geometry_msgs::msg::Vector3 hand_offset;
    geometry_msgs::msg::Quaternion handover_orientation;
    double lift_height = 0.0;
};

struct MotionProfiles {
    std::vector<double> home_joints;
    std::vector<double> long_scissor_over_joints;
    std::vector<double> long_scissor_after_joints;
    double long_scissor_elbow_target = std::numeric_limits<double>::quiet_NaN();
    double long_scissor_elbow_tolerance_above = std::numeric_limits<double>::quiet_NaN();
    double long_scissor_elbow_tolerance_below = std::numeric_limits<double>::quiet_NaN();
};

class HandPositionFollower : public rclcpp::Node {
public:
    HandPositionFollower()
    : Node("moveit_mover") {
        // OMPL Parameter deklarieren
        this->declare_parameter("robot_description_planning.default_planner_config", "RRTConnect");
        this->declare_parameter("robot_description_planning.default_planner_config.settings", "RRTConnectkConfigDefault");
        // Optional: Timeout und Planungszeit konfigurieren
        this->declare_parameter("robot_description_planning.ompl_planning.timeout", 10.0);
        this->declare_parameter("robot_description_planning.ompl_planning.max_planning_attempts", 10);
        // Kinematik-Plugin deklarieren
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_timeout", 0.05);

        try {
            declareConfigurationParameters();
            configuration_loaded_ = loadConfiguration();
        } catch (const rclcpp::exceptions::InvalidParameterTypeException &ex) {
            configuration_loaded_ = false;
            RCLCPP_ERROR(this->get_logger(), "Configuration load failed due to parameter type mismatch: %s", ex.what());
        } catch (const std::exception &ex) {
            configuration_loaded_ = false;
            RCLCPP_ERROR(this->get_logger(), "Configuration load failed: %s", ex.what());
        }

        // Abonnieren des /hand_position Topics
        hand_position_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/hand_pose",
            10,
            std::bind(&HandPositionFollower::handPositionCallback, this, std::placeholders::_1));
        // Gripper-Status abonnieren
        gripper_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/gripper_done",
            10,
            std::bind(&HandPositionFollower::gripperDoneCallback, this, std::placeholders::_1));
        // tool_selection abonnieren
        tool_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/tool_selection",
            10,
            std::bind(&HandPositionFollower::toolSelectionCallback, this, std::placeholders::_1));
        // joint_state_buffered abonnieren
        joint_state_buffered_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state_buffered",
            10,
            std::bind(&HandPositionFollower::jointStateBufferedCallback, this, std::placeholders::_1));

        // Publisher initialisieren
        gripper_mover_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);
        gripper_zeroer_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_zeroer", 10);
        request_joint_state_ = this->create_publisher<std_msgs::msg::Bool>("/request_joint_state", 10);
        handover_event_publisher_ = this->create_publisher<std_msgs::msg::String>("/handover_event", 10);

        RCLCPP_INFO(this->get_logger(), "Moveit Mover Node initialized.");
    }

    // Initialisiere die MoveGroupInterface
    void initializeMoveGroupInterface() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");
        move_group_->setEndEffectorLink("gripper_tip_link");

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");

        if (!configuration_loaded_) {
            RCLCPP_ERROR(this->get_logger(), "Motion configuration is invalid. Rejecting motion execution until configuration is fixed.");
            return;
        }

        moveToHomePositionUsingJoints();
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        publishGripperMover(true);
    }

private:
    geometry_msgs::msg::Point tool_position_;
    geometry_msgs::msg::Quaternion tool_orientation_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    geometry_msgs::msg::Pose hand_pose_;
    geometry_msgs::msg::Pose hand_pose_with_offset_;
    geometry_msgs::msg::Vector3 hand_offset_;
    sensor_msgs::msg::JointState latest_joint_state_;
    ToolProfile active_tool_profile_;
    MotionProfiles motion_profiles_;
    std::unordered_map<std::string, ToolProfile> tool_profiles_;
    bool hand_pose_received_ = false;
    bool waiting_for_hand_pose_ = false;
    bool waiting_for_gripper_done_ = false;
    bool tool_has_been_picked_up_ = false;
    bool received_joint_state_ = false;
    bool configuration_loaded_ = false;

    void declareConfigurationParameters() {
        this->declare_parameter<std::vector<std::string>>("tool_ids", std::vector<std::string>{});
        this->declare_parameter<std::vector<double>>("motion_profiles.home_joints", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("motion_profiles.long_scissor.over_joints", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("motion_profiles.long_scissor.after_joints", std::vector<double>{});
        this->declare_parameter("motion_profiles.long_scissor.elbow_target", std::numeric_limits<double>::quiet_NaN());
        this->declare_parameter("motion_profiles.long_scissor.elbow_tolerance_above", std::numeric_limits<double>::quiet_NaN());
        this->declare_parameter("motion_profiles.long_scissor.elbow_tolerance_below", std::numeric_limits<double>::quiet_NaN());

        std::vector<std::string> tool_ids;
        this->get_parameter("tool_ids", tool_ids);
        for (const std::string &tool_id : tool_ids) {
            declareToolProfileParameters(tool_id);
        }
    }

    void declareToolProfileParameters(const std::string &tool_id) {
        const std::string prefix = "tool_profiles." + tool_id + ".";

        if (!this->has_parameter(prefix + "strategy")) {
            this->declare_parameter(prefix + "strategy", std::string());
        }
        if (!this->has_parameter(prefix + "pick_position")) {
            this->declare_parameter<std::vector<double>>(prefix + "pick_position", std::vector<double>{});
        }
        if (!this->has_parameter(prefix + "pick_orientation")) {
            this->declare_parameter<std::vector<double>>(prefix + "pick_orientation", std::vector<double>{});
        }
        if (!this->has_parameter(prefix + "hand_offset")) {
            this->declare_parameter<std::vector<double>>(prefix + "hand_offset", std::vector<double>{});
        }
        if (!this->has_parameter(prefix + "handover_orientation")) {
            this->declare_parameter<std::vector<double>>(prefix + "handover_orientation", std::vector<double>{});
        }
        if (!this->has_parameter(prefix + "lift_height")) {
            this->declare_parameter(prefix + "lift_height", std::numeric_limits<double>::quiet_NaN());
        }
    }

    bool loadConfiguration() {
        std::vector<std::string> tool_ids;
        if (!this->get_parameter("tool_ids", tool_ids) || tool_ids.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Missing or empty required parameter: tool_ids");
            return false;
        }

        MotionProfiles loaded_motion_profiles;
        std::unordered_map<std::string, ToolProfile> loaded_tool_profiles;

        if (!getDoubleArrayParameter("motion_profiles.home_joints", 6, loaded_motion_profiles.home_joints)) {
            return false;
        }
        if (!getDoubleArrayParameter(
                "motion_profiles.long_scissor.over_joints",
                6,
                loaded_motion_profiles.long_scissor_over_joints)) {
            return false;
        }
        if (!getDoubleArrayParameter(
                "motion_profiles.long_scissor.after_joints",
                6,
                loaded_motion_profiles.long_scissor_after_joints)) {
            return false;
        }
        if (!getDoubleParameter("motion_profiles.long_scissor.elbow_target", loaded_motion_profiles.long_scissor_elbow_target)) {
            return false;
        }
        if (!getDoubleParameter(
                "motion_profiles.long_scissor.elbow_tolerance_above",
                loaded_motion_profiles.long_scissor_elbow_tolerance_above)) {
            return false;
        }
        if (!getDoubleParameter(
                "motion_profiles.long_scissor.elbow_tolerance_below",
                loaded_motion_profiles.long_scissor_elbow_tolerance_below)) {
            return false;
        }

        for (const std::string &tool_id : tool_ids) {
            ToolProfile profile;
            if (!loadToolProfile(tool_id, profile)) {
                return false;
            }
            loaded_tool_profiles[tool_id] = profile;
        }

        motion_profiles_ = loaded_motion_profiles;
        tool_profiles_ = std::move(loaded_tool_profiles);

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu tool profiles from ROS2 parameters.",
            tool_profiles_.size());
        return true;
    }

    bool loadToolProfile(const std::string &tool_id, ToolProfile &out_profile) {
        declareToolProfileParameters(tool_id);
        const std::string prefix = "tool_profiles." + tool_id + ".";

        std::string strategy_name;
        if (!this->get_parameter(prefix + "strategy", strategy_name) || strategy_name.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Missing or empty required parameter: %s", (prefix + "strategy").c_str());
            return false;
        }

        std::vector<double> pick_position;
        if (!getDoubleArrayParameter(prefix + "pick_position", 3, pick_position)) {
            return false;
        }

        std::vector<double> pick_orientation;
        if (!getDoubleArrayParameter(prefix + "pick_orientation", 4, pick_orientation)) {
            return false;
        }

        std::vector<double> hand_offset;
        if (!getDoubleArrayParameter(prefix + "hand_offset", 3, hand_offset)) {
            return false;
        }

        std::vector<double> handover_orientation;
        if (!getDoubleArrayParameter(prefix + "handover_orientation", 4, handover_orientation)) {
            return false;
        }

        double lift_height = std::numeric_limits<double>::quiet_NaN();
        if (!getDoubleParameter(prefix + "lift_height", lift_height)) {
            return false;
        }

        try {
            out_profile.tool_id = tool_id;
            out_profile.strategy = parseGrabStrategy(strategy_name);
            out_profile.pick_position = pointFromArray(pick_position);
            out_profile.pick_orientation = quaternionFromArray(pick_orientation);
            out_profile.hand_offset = vector3FromArray(hand_offset);
            out_profile.handover_orientation = quaternionFromArray(handover_orientation);
            out_profile.lift_height = lift_height;
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Invalid tool profile for tool id '%s': %s", tool_id.c_str(), ex.what());
            return false;
        }

        return true;
    }

    bool getDoubleArrayParameter(const std::string &name, size_t expected_size, std::vector<double> &out) {
        if (!this->get_parameter(name, out)) {
            RCLCPP_ERROR(this->get_logger(), "Missing required parameter: %s", name.c_str());
            return false;
        }

        if (out.size() != expected_size) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter %s must contain exactly %zu values but has %zu.",
                name.c_str(),
                expected_size,
                out.size());
            return false;
        }

        return true;
    }

    bool getDoubleParameter(const std::string &name, double &out) {
        if (!this->get_parameter(name, out) || !std::isfinite(out)) {
            RCLCPP_ERROR(this->get_logger(), "Missing or invalid required parameter: %s", name.c_str());
            return false;
        }

        return true;
    }

    geometry_msgs::msg::Point pointFromArray(const std::vector<double> &values) {
        if (values.size() != 3) {
            throw std::invalid_argument("point arrays must have 3 values");
        }

        geometry_msgs::msg::Point point;
        point.x = values[0];
        point.y = values[1];
        point.z = values[2];
        return point;
    }

    geometry_msgs::msg::Quaternion quaternionFromArray(const std::vector<double> &values) {
        if (values.size() != 4) {
            throw std::invalid_argument("quaternion arrays must have 4 values");
        }

        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = values[0];
        quaternion.y = values[1];
        quaternion.z = values[2];
        quaternion.w = values[3];
        return quaternion;
    }

    geometry_msgs::msg::Vector3 vector3FromArray(const std::vector<double> &values) {
        if (values.size() != 3) {
            throw std::invalid_argument("vector arrays must have 3 values");
        }

        geometry_msgs::msg::Vector3 vector;
        vector.x = values[0];
        vector.y = values[1];
        vector.z = values[2];
        return vector;
    }

    GrabStrategy parseGrabStrategy(const std::string &value) {
        if (value == "standard") {
            return GrabStrategy::Standard;
        }
        if (value == "hammer_cartesian") {
            return GrabStrategy::HammerCartesian;
        }
        if (value == "long_scissor") {
            return GrabStrategy::LongScissor;
        }
        if (value == "retractor_cartesian") {
            return GrabStrategy::RetractorCartesian;
        }

        throw std::invalid_argument("unknown strategy '" + value + "'");
    }

    void activateToolProfile(const ToolProfile &profile) {
        active_tool_profile_ = profile;
        tool_position_ = profile.pick_position;
        tool_orientation_ = profile.pick_orientation;
        hand_offset_ = profile.hand_offset;
        handover_orientation_ = profile.handover_orientation;
    }

    void handPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!waiting_for_hand_pose_) {
            RCLCPP_INFO(this->get_logger(), "Action rejected: hand pose received but system is not waiting for a gesture.");
            return;
        }

        if (!tool_has_been_picked_up_) {
            RCLCPP_INFO(this->get_logger(), "Action rejected: hand pose received but no tool has been picked up yet.");
            return;
        }

        hand_pose_ = *msg;
        hand_pose_with_offset_ = *msg;
        hand_pose_with_offset_.position.x += hand_offset_.x;
        hand_pose_with_offset_.position.y += hand_offset_.y;
        hand_pose_with_offset_.position.z += hand_offset_.z;
        publishHandoverEvent("gesture_detected");
        RCLCPP_INFO(this->get_logger(), "Gesture detected from hand pose message, proceeding with handover...");
        performHandoverToHandPose();
    }

    void toolSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string cmd = msg->data;

        if (cmd == "0") {
            RCLCPP_WARN(this->get_logger(), "RESETTING SYSTEM STATE MANUALLY.");
            hand_pose_received_ = false;
            waiting_for_hand_pose_ = false;
            waiting_for_gripper_done_ = false;
            tool_has_been_picked_up_ = false;
            publishGripperMover(true);
            publishGripperZeroer(false);

            if (move_group_ != nullptr && configuration_loaded_) {
                moveToHomePositionUsingJoints();
            } else {
                RCLCPP_WARN(this->get_logger(), "Reset skipped home motion because move group or configuration is unavailable.");
            }

            RCLCPP_INFO(this->get_logger(), "System reset complete.");
            return;
        }

        if (!configuration_loaded_) {
            RCLCPP_ERROR(this->get_logger(), "Action rejected: tool command '%s' received but motion configuration is invalid.", cmd.c_str());
            return;
        }

        if (move_group_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Action rejected: tool command '%s' received before MoveGroupInterface initialization.", cmd.c_str());
            return;
        }

        if (waiting_for_hand_pose_) {
            RCLCPP_WARN(this->get_logger(), "Already waiting for hand pose - ignoring tool command.");
            return;
        }

        const auto profile_it = tool_profiles_.find(cmd);
        if (profile_it == tool_profiles_.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", cmd.c_str());
            return;
        }

        activateToolProfile(profile_it->second);
        RCLCPP_INFO(this->get_logger(), "Action accepted: tool %s selected with configured strategy.", cmd.c_str());

        switch (profile_it->second.strategy) {
            case GrabStrategy::Standard:
                grabTool(tool_position_.x, tool_position_.y, tool_position_.z);
                break;
            case GrabStrategy::HammerCartesian:
                grabHammer(tool_position_.x, tool_position_.y, tool_position_.z);
                break;
            case GrabStrategy::LongScissor:
                grabLongScissor(tool_position_.x, tool_position_.y, tool_position_.z);
                break;
            case GrabStrategy::RetractorCartesian:
                grabRetractor(tool_position_.x, tool_position_.y, tool_position_.z);
                break;
        }

        if (waiting_for_hand_pose_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for hand pose...");
        } else {
            RCLCPP_WARN(this->get_logger(), "Action rejected: tool %s did not complete pickup, see previous planning errors.", cmd.c_str());
        }
    }

    void performHandoverToHandPose() {
        geometry_msgs::msg::Pose target_pose = hand_pose_with_offset_;
        target_pose.orientation = handover_orientation_;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.6);
        move_group_->setMaxAccelerationScalingFactor(0.6);
        move_group_->setPoseTarget(target_pose);
        RCLCPP_INFO(
            this->get_logger(),
            "Target for handover: x=%.2f y=%.2f z=%.2f",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Reachability decision: reachable.");
            RCLCPP_INFO(this->get_logger(), "Moving above hand...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            RCLCPP_INFO(this->get_logger(), "Activating gripper sensing...");
            publishGripperZeroer(true);
            waiting_for_gripper_done_ = true;
            hand_pose_received_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        } else {
            publishHandoverEvent("reachability:unreachable_plan_failed");
            RCLCPP_ERROR(this->get_logger(), "Reachability decision: unreachable.");
            RCLCPP_ERROR(this->get_logger(), "Planning to above hand failed.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tool_selection_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_;

    void gripperDoneCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Gripper opened, returning to home.");
            moveToHomePositionUsingJoints();
            hand_pose_received_ = false;
            waiting_for_hand_pose_ = false;
            tool_has_been_picked_up_ = false;
            RCLCPP_INFO(this->get_logger(), "Ready for next tool command.");
        }
    }

    void jointStateBufferedCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = *msg;
        received_joint_state_ = true;
        RCLCPP_INFO(this->get_logger(), "Received joint_state from Python relay.");
    }

    void moveToHomePositionUsingJoints() {
        if (move_group_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Cannot move home: MoveGroupInterface is not initialized.");
            return;
        }

        if (motion_profiles_.home_joints.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Cannot move home: configured home joint preset is invalid.");
            return;
        }

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(motion_profiles_.home_joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to Home-Position (Joints) successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to Home-Position (Joints) failed.");
        }
    }

    void grabTool(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        publishGripperMover(true);

        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;

        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += active_tool_profile_.lift_height;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(lift_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(object_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object failed.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        tool_has_been_picked_up_ = true;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(lift_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Lifting object...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Lift motion failed.");
        }
    }

    void grabHammer(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        publishGripperMover(true);

        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;

        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += active_tool_profile_.lift_height;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(lift_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(object_pose);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(object_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        const double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Linear path to object (%.2f%% achieved), executing...", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_->execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute linear Cartesian path (only %.2f%% achieved)", fraction * 100.0);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        tool_has_been_picked_up_ = true;

        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_pose);

        moveit_msgs::msg::RobotTrajectory lift_trajectory;
        const double lift_fraction = move_group_->computeCartesianPath(
            lift_waypoints, eef_step, jump_threshold, lift_trajectory);

        if (lift_fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Lifting object (linear path %.2f%% achieved)...", lift_fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            lift_plan.trajectory_ = lift_trajectory;
            move_group_->execute(lift_plan);

            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian lift motion failed (%.2f%% achieved)", lift_fraction * 100.0);
        }
    }

    void grabLongScissor(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        publishGripperMover(true);

        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(motion_profiles_.long_scissor_over_joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
            return;
        }

        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::JointConstraint elbow_constraint;
        elbow_constraint.joint_name = "elbow_joint";
        elbow_constraint.position = motion_profiles_.long_scissor_elbow_target;
        elbow_constraint.tolerance_above = motion_profiles_.long_scissor_elbow_tolerance_above;
        elbow_constraint.tolerance_below = motion_profiles_.long_scissor_elbow_tolerance_below;
        elbow_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(elbow_constraint);

        move_group_->setPathConstraints(constraints);
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(object_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planned motion to object pose successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object pose failed.");
            move_group_->clearPathConstraints();
            return;
        }
        move_group_->clearPathConstraints();

        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        tool_has_been_picked_up_ = true;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(motion_profiles_.long_scissor_after_joints);

        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
            return;
        }

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(motion_profiles_.long_scissor_over_joints);

        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
        }
    }

    void grabRetractor(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        publishGripperMover(true);

        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;

        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += active_tool_profile_.lift_height;

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(lift_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }

        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setPoseTarget(object_pose);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(object_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        const double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Linear path to object (%.2f%% achieved), executing...", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_->execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute linear Cartesian path (only %.2f%% achieved)", fraction * 100.0);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        tool_has_been_picked_up_ = true;

        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_pose);

        moveit_msgs::msg::RobotTrajectory lift_trajectory;
        const double lift_fraction = move_group_->computeCartesianPath(
            lift_waypoints, eef_step, jump_threshold, lift_trajectory);

        if (lift_fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Lifting object (linear path %.2f%% achieved)...", lift_fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            lift_plan.trajectory_ = lift_trajectory;
            move_group_->execute(lift_plan);

            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian lift motion failed (%.2f%% achieved)", lift_fraction * 100.0);
        }
    }

    void moveToModifiedJoints() {
        received_joint_state_ = false;
        publishRequestJointState(true);
        RCLCPP_INFO(this->get_logger(), "Published joint state request.");

        std::thread([this]() {
            int retries = 0;
            while (!received_joint_state_ && rclcpp::ok() && retries < 30) {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_WARN(this->get_logger(), "Waiting for joint state... (%d)", retries);
                ++retries;
            }

            if (!received_joint_state_) {
                RCLCPP_ERROR(this->get_logger(), "Still no joint state received.");
                return;
            }

            this->planAndExecuteFromBufferedState();
        }).detach();
    }

    void planAndExecuteFromBufferedState() {
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
            joint_map[latest_joint_state_.name[i]] = latest_joint_state_.position[i];
        }

        joint_map["wrist_2_joint"] = 0.0;
        joint_map["wrist_3_joint"] = 0.0;

        const std::vector<std::string> ur_joint_order = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        std::vector<double> joint_positions;
        constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
        for (const std::string &name : ur_joint_order) {
            const double rad = joint_map[name];
            const double deg = rad * rad_to_deg;
            RCLCPP_INFO(this->get_logger(), "Target Joint %s: %.4f rad (%.2f deg)", name.c_str(), rad, deg);
            joint_positions.push_back(rad);
        }

        move_group_->setPlanningTime(0.5);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Modified joint movement planned successfully, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }

    void publishGripperMover(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_mover_->publish(msg);
    }

    void publishGripperZeroer(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_zeroer_->publish(msg);
    }

    void publishRequestJointState(bool trigger) {
        auto msg = std_msgs::msg::Bool();
        msg.data = trigger;
        request_joint_state_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published joint state request.");
    }

    void publishHandoverEvent(const std::string &event_name) {
        auto msg = std_msgs::msg::String();
        msg.data = event_name;
        handover_event_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published handover event: %s", event_name.c_str());
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_position_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_buffered_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_joint_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr handover_event_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HandPositionFollower>();
    node->initializeMoveGroupInterface();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
