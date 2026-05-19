// Skill Executor Node
// ===================
// Provides four ROS actions that wrap the MoveIt-based pick / handover /
// release / return_home skills from tool_pick_test_node.cpp, plus publishes
// /system_state_update so world_model_node can surface the current state.
//
// Actions:
//   /pick_tool       (tracking_pkg/action/PickTool)
//   /handover_tool   (tracking_pkg/action/HandoverTool)
//   /release_tool    (tracking_pkg/action/ReleaseTool)
//   /return_home     (tracking_pkg/action/ReturnHome)
//
// State updates on /system_state_update use the convention
//   "STATE:tool_id:tool_class" parsed by world_model_node._state_cb.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <tracking_pkg/msg/grasp_candidate.hpp>
#include <tracking_pkg/msg/hand_state.hpp>
#include <tracking_pkg/srv/get_world_state.hpp>
#include <tracking_pkg/action/pick_tool.hpp>
#include <tracking_pkg/action/handover_tool.hpp>
#include <tracking_pkg/action/release_tool.hpp>
#include <tracking_pkg/action/return_home.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {

using PickTool = tracking_pkg::action::PickTool;
using HandoverTool = tracking_pkg::action::HandoverTool;
using ReleaseTool = tracking_pkg::action::ReleaseTool;
using ReturnHome = tracking_pkg::action::ReturnHome;

using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickTool>;
using GoalHandleHandover = rclcpp_action::ServerGoalHandle<HandoverTool>;
using GoalHandleRelease = rclcpp_action::ServerGoalHandle<ReleaseTool>;
using GoalHandleHome = rclcpp_action::ServerGoalHandle<ReturnHome>;

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
        name.c_str(), value.size(), expected_size);
    return fallback;
}

bool isFinitePoint(const geometry_msgs::msg::Point &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool hasUsableWorldXy(const tracking_pkg::msg::GraspCandidate &c) {
    if (!isFinitePoint(c.grasp_pose.pose.position)) return false;
    if (c.grasp_pose.header.frame_id != "world") return false;
    return std::hypot(c.grasp_pose.pose.position.x, c.grasp_pose.pose.position.y) > 1e-4;
}

moveit_msgs::msg::RobotState makeStartStateFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    moveit_msgs::msg::RobotState rs;
    rs.is_diff = false;
    rs.joint_state.name = plan.trajectory_.joint_trajectory.joint_names;
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
        rs.joint_state.position = plan.trajectory_.joint_trajectory.points.back().positions;
    }
    return rs;
}

}  // namespace


class SkillExecutor : public rclcpp::Node {
public:
    SkillExecutor()
    : Node("skill_executor_node") {
        // ── Parameters (same defaults as tool_pick_test_node.cpp) ──────
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
        gripper_done_timeout_seconds_ = declare_parameter("gripper_done_timeout_seconds", 30.0);
        cartesian_min_fraction_ = declare_parameter("cartesian_min_fraction", 0.95);

        joint_state_names_ = declare_parameter(
            "joint_state_names",
            std::vector<std::string>{
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            });
        if (joint_state_names_.size() != 6) {
            RCLCPP_WARN(get_logger(),
                "joint_state_names has %zu entries (expected 6); using UR defaults.",
                joint_state_names_.size());
            joint_state_names_ = {
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
            };
        }

        const auto ho = parameterVectorOrDefault(
            *this, "hand_offset", {-0.08, 0.0, 0.05}, 3);
        hand_offset_.x = ho[0]; hand_offset_.y = ho[1]; hand_offset_.z = ho[2];

        const auto hq = parameterVectorOrDefault(
            *this, "handover_orientation", {-0.63, 0.63, -0.321, 0.321}, 4);
        handover_orientation_.x = hq[0]; handover_orientation_.y = hq[1];
        handover_orientation_.z = hq[2]; handover_orientation_.w = hq[3];

        home_joints_ = parameterVectorOrDefault(
            *this, "home_joints",
            {-0.1601136366, -2.2975937329, 2.2748802344,
             -1.5248240244, -1.2305892150, -4.8166621367},
            6);

        // ── Publishers ───────────────────────────────────────────────
        gripper_mover_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);
        gripper_zeroer_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper_zeroer", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/system_state_update", 10);

        // ── Subscribers ──────────────────────────────────────────────
        hand_state_sub_ = create_subscription<tracking_pkg::msg::HandState>(
            "/hand_state", 10,
            std::bind(&SkillExecutor::handStateCb, this, std::placeholders::_1));
        gripper_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/gripper_done", 10,
            std::bind(&SkillExecutor::gripperDoneCb, this, std::placeholders::_1));

        // ── Service client ───────────────────────────────────────────
        world_state_client_ =
            create_client<tracking_pkg::srv::GetWorldState>("/get_world_state");

        RCLCPP_INFO(get_logger(),
            "SkillExecutor constructed (move_group=%s, ee=%s, frame=%s).",
            move_group_name_.c_str(), end_effector_link_.c_str(),
            reference_frame_.c_str());
    }

    // Must be called from main() after make_shared, because MoveGroupInterface
    // needs shared_from_this(). Also registers action servers (so MoveIt is
    // guaranteed available when the first action fires).
    void initialize() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), move_group_name_);
        move_group_->setEndEffectorLink(end_effector_link_);
        move_group_->setPoseReferenceFrame(reference_frame_);
        move_group_->setPlanningTime(2.0);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        pick_srv_ = rclcpp_action::create_server<PickTool>(
            this, "pick_tool",
            std::bind(&SkillExecutor::pickHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::pickHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::pickHandleAccepted, this,
                      std::placeholders::_1));

        handover_srv_ = rclcpp_action::create_server<HandoverTool>(
            this, "handover_tool",
            std::bind(&SkillExecutor::handoverHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::handoverHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::handoverHandleAccepted, this,
                      std::placeholders::_1));

        release_srv_ = rclcpp_action::create_server<ReleaseTool>(
            this, "release_tool",
            std::bind(&SkillExecutor::releaseHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::releaseHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::releaseHandleAccepted, this,
                      std::placeholders::_1));

        home_srv_ = rclcpp_action::create_server<ReturnHome>(
            this, "return_home",
            std::bind(&SkillExecutor::homeHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::homeHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::homeHandleAccepted, this,
                      std::placeholders::_1));

        publishState("IDLE", "", "");
        RCLCPP_INFO(get_logger(), "SkillExecutor action servers ready.");
    }

private:
    // ── Callbacks ────────────────────────────────────────────────────

    void handStateCb(const tracking_pkg::msg::HandState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_hand_state_ = *msg;
        have_hand_state_ = true;
    }

    void gripperDoneCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        {
            std::lock_guard<std::mutex> lock(gripper_done_mutex_);
            gripper_done_received_ = true;
        }
        gripper_done_cv_.notify_all();
    }

    // ── State publish ────────────────────────────────────────────────

    void publishState(const std::string &state,
                      const std::string &tool_id,
                      const std::string &tool_class) {
        std_msgs::msg::String msg;
        msg.data = state + ":" + tool_id + ":" + tool_class;
        state_pub_->publish(msg);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = state;
            if (state == "PICKING" || state == "TRANSPORTING"
                || state == "HANDOVER" || state == "RELEASING") {
                active_tool_id_ = tool_id;
                active_tool_class_ = tool_class;
            } else if (state == "IDLE") {
                active_tool_id_ = "";
                active_tool_class_ = "";
            }
        }
    }

    // ── World-state lookup ───────────────────────────────────────────

    bool fetchCandidates(std::vector<tracking_pkg::msg::GraspCandidate> &out,
                         std::string &err) {
        if (!world_state_client_->wait_for_service(std::chrono::seconds(2))) {
            err = "/get_world_state service not available";
            return false;
        }
        auto req = std::make_shared<tracking_pkg::srv::GetWorldState::Request>();
        auto future = world_state_client_->async_send_request(req);
        // We're running in a separate thread (action execute); spinning the
        // node ourselves would deadlock with the main executor. Instead wait
        // on the future with a timeout — the executor in main() will service
        // the response.
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            err = "/get_world_state timed out";
            return false;
        }
        auto resp = future.get();
        if (!resp->success) {
            err = "/get_world_state returned !success: " + resp->message;
            return false;
        }
        out = resp->world_state.tool_candidates;
        return true;
    }

    bool collectCandidates(std::vector<tracking_pkg::msg::GraspCandidate> &out_sorted,
                           std::string &err) {
        std::vector<tracking_pkg::msg::GraspCandidate> all;
        if (!fetchCandidates(all, err)) return false;
        for (const auto &c : all) {
            if (hasUsableWorldXy(c)) out_sorted.push_back(c);
        }
        std::sort(out_sorted.begin(), out_sorted.end(),
                  [](const auto &a, const auto &b) {
                      return a.grasp_confidence > b.grasp_confidence;
                  });
        if (out_sorted.empty()) {
            err = "no usable candidates from world model";
            return false;
        }
        return true;
    }

    // ── MoveIt primitives (ported from tool_pick_test_node.cpp) ──────

    bool moveToPoseTarget(const geometry_msgs::msg::Pose &pose,
                          std::string &err) {
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "planning to pose target failed";
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "execution to pose target failed";
            return false;
        }
        return true;
    }

    bool moveLinearToPose(const geometry_msgs::msg::Pose &pose,
                          std::string &err) {
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        std::vector<geometry_msgs::msg::Pose> waypoints{pose};
        moveit_msgs::msg::RobotTrajectory traj;
        const double fraction = move_group_->computeCartesianPath(
            waypoints, 0.005, 0.0, traj);
        if (fraction < cartesian_min_fraction_) {
            err = "cartesian path only " + std::to_string(fraction * 100.0) + "%";
            return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "cartesian path execution failed";
            return false;
        }
        return true;
    }

    // ── Plan-only primitives (used for pre-flight pick validation) ───
    // Caller is responsible for setStartState before calling.

    bool planPoseTarget(const geometry_msgs::msg::Pose &pose,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan_out,
                        std::string &err) {
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setPoseTarget(pose);
        if (move_group_->plan(plan_out) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "planning to pose target failed";
            move_group_->clearPoseTargets();
            return false;
        }
        move_group_->clearPoseTargets();
        return true;
    }

    bool planLinearPose(const geometry_msgs::msg::Pose &pose,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan_out,
                        std::string &err) {
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        std::vector<geometry_msgs::msg::Pose> waypoints{pose};
        moveit_msgs::msg::RobotTrajectory traj;
        const double fraction = move_group_->computeCartesianPath(
            waypoints, 0.005, 0.0, traj);
        if (fraction < cartesian_min_fraction_) {
            err = "cartesian path only " + std::to_string(fraction * 100.0) + "%";
            return false;
        }
        plan_out.trajectory_ = traj;
        return true;
    }

    bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                     std::string &err) {
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "execute failed";
            return false;
        }
        return true;
    }

    bool moveToJointPositions(const std::vector<double> &joints,
                              std::string &err) {
        if (joint_state_names_.size() != joints.size()) {
            err = "joint name/value size mismatch";
            return false;
        }
        std::map<std::string, double> target;
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            target[joint_state_names_[i]] = joints[i];
        }
        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(2.0);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "joint plan failed";
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "joint execute failed";
            return false;
        }
        return true;
    }

    // ── Gripper primitives ───────────────────────────────────────────

    void publishGripper(bool open) {
        std_msgs::msg::Bool m;
        m.data = open;
        gripper_mover_pub_->publish(m);
    }

    void publishGripperZeroer(bool active) {
        std_msgs::msg::Bool m;
        m.data = active;
        gripper_zeroer_pub_->publish(m);
    }

    void sleepForGripper() {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, gripper_pause_seconds_))));
    }

    bool waitForGripperDone(double timeout_s) {
        std::unique_lock<std::mutex> lock(gripper_done_mutex_);
        gripper_done_received_ = false;
        const bool ok = gripper_done_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(timeout_s)),
            [this] { return gripper_done_received_; });
        return ok;
    }

    // ── Skill: PickTool ──────────────────────────────────────────────

    bool tryPlanPickSequence(
        const tracking_pkg::msg::GraspCandidate &cand,
        moveit::planning_interface::MoveGroupInterface::Plan &approach_plan,
        moveit::planning_interface::MoveGroupInterface::Plan &descend_plan,
        moveit::planning_interface::MoveGroupInterface::Plan &lift_plan,
        geometry_msgs::msg::Pose &approach_pose_out,
        geometry_msgs::msg::Pose &grasp_pose_out,
        std::string &err) {
        grasp_pose_out.position.x = cand.grasp_pose.pose.position.x;
        grasp_pose_out.position.y = cand.grasp_pose.pose.position.y;
        grasp_pose_out.position.z = cand.grasp_pose.pose.position.z + z_offset_m_;
        grasp_pose_out.orientation = topDownQuaternionFromHandleAxis(
            cand.handle_axis, tool_yaw_offset_rad_);
        approach_pose_out = grasp_pose_out;
        approach_pose_out.position.z = grasp_pose_out.position.z + approach_height_m_;

        // 1. Plan approach from current state
        move_group_->setStartStateToCurrentState();
        if (!planPoseTarget(approach_pose_out, approach_plan, err)) {
            err = "approach plan: " + err;
            return false;
        }

        // 2. Plan descend (cartesian) from approach end state
        move_group_->setStartState(makeStartStateFromPlanEnd(approach_plan));
        if (!planLinearPose(grasp_pose_out, descend_plan, err)) {
            err = "descend plan: " + err;
            return false;
        }

        // 3. Plan lift (cartesian) from descend end state
        move_group_->setStartState(makeStartStateFromPlanEnd(descend_plan));
        if (!planLinearPose(approach_pose_out, lift_plan, err)) {
            err = "lift plan: " + err;
            return false;
        }

        return true;
    }

    bool doPick(const std::string &tool_id_arg,
                std::string &picked_id_out,
                std::string &picked_class_out,
                std::string &err) {
        std::vector<tracking_pkg::msg::GraspCandidate> candidates;
        if (!collectCandidates(candidates, err)) return false;

        // Restrict to a specific tool_id when explicitly requested.
        if (!tool_id_arg.empty()) {
            candidates.erase(
                std::remove_if(candidates.begin(), candidates.end(),
                    [&](const auto &c) { return c.tool_id != tool_id_arg; }),
                candidates.end());
            if (candidates.empty()) {
                err = "tool_id '" + tool_id_arg + "' not in candidates";
                return false;
            }
        }

        // Pre-flight: try each candidate in confidence order. Only when
        // approach + descend + lift all plan successfully do we commit to
        // a pick. No motion happens before this loop succeeds.
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan, descend_plan, lift_plan;
        geometry_msgs::msg::Pose approach_pose, grasp_pose;
        tracking_pkg::msg::GraspCandidate chosen;
        std::vector<std::string> rejection_log;
        bool found = false;
        for (const auto &cand : candidates) {
            std::string plan_err;
            if (tryPlanPickSequence(cand, approach_plan, descend_plan, lift_plan,
                                    approach_pose, grasp_pose, plan_err)) {
                chosen = cand;
                found = true;
                break;
            }
            rejection_log.push_back(
                cand.tool_id + " (" + cand.tool_class + "): " + plan_err);
            RCLCPP_WARN(get_logger(), "Pre-flight rejected %s: %s",
                        cand.tool_id.c_str(), plan_err.c_str());
        }
        if (!found) {
            err = "no reachable candidate. Tried " +
                  std::to_string(rejection_log.size()) + ": ";
            for (const auto &r : rejection_log) err += "[" + r + "] ";
            return false;
        }

        picked_id_out = chosen.tool_id;
        picked_class_out = chosen.tool_class;

        RCLCPP_INFO(get_logger(),
            "Pre-flight OK for %s (%s). grasp z=%.4f approach z=%.4f. Executing.",
            chosen.tool_id.c_str(), chosen.tool_class.c_str(),
            grasp_pose.position.z, approach_pose.position.z);

        publishState("PICKING", chosen.tool_id, chosen.tool_class);

        if (!executePlan(approach_plan, err)) { err = "approach exec: " + err; return false; }
        publishGripper(true);   // open
        sleepForGripper();
        if (!executePlan(descend_plan, err))  { err = "descend exec: "  + err; return false; }
        publishGripper(false);  // close
        sleepForGripper();
        if (!executePlan(lift_plan, err))     { err = "lift exec: "     + err; return false; }

        publishState("TRANSPORTING", chosen.tool_id, chosen.tool_class);
        return true;
    }

    // ── Skill: HandoverTool ──────────────────────────────────────────

    bool doHandover(const geometry_msgs::msg::PoseStamped &goal_pose,
                    std::string &err) {
        geometry_msgs::msg::Pose hand_pose;
        bool have_pose = false;

        // Honor explicit goal pose if frame is set, else fall back to last
        // /hand_state.
        if (!goal_pose.header.frame_id.empty()) {
            hand_pose = goal_pose.pose;
            have_pose = true;
        } else {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_hand_state_ && last_hand_state_.is_tracked) {
                hand_pose = last_hand_state_.hand_pose.pose;
                have_pose = true;
            }
        }
        if (!have_pose) {
            err = "no hand pose available (no goal pose and last /hand_state not tracked)";
            return false;
        }

        std::string tool_id_snapshot, tool_class_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            tool_id_snapshot = active_tool_id_;
            tool_class_snapshot = active_tool_class_;
        }

        publishState("HANDOVER", tool_id_snapshot, tool_class_snapshot);

        geometry_msgs::msg::Pose target = hand_pose;
        target.position.x += hand_offset_.x;
        target.position.y += hand_offset_.y;
        target.position.z += hand_offset_.z;
        target.orientation = handover_orientation_;

        RCLCPP_INFO(get_logger(),
            "HandoverTool: planning to (%.3f, %.3f, %.3f)",
            target.position.x, target.position.y, target.position.z);

        move_group_->setStartStateToCurrentState();
        move_group_->setPlanningTime(handover_planning_time_);
        move_group_->setMaxVelocityScalingFactor(handover_velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(handover_acceleration_scale_);
        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->clearPoseTargets();
            err = "handover plan failed";
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->clearPoseTargets();
            err = "handover execute failed";
            return false;
        }
        move_group_->clearPoseTargets();

        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, pre_release_dwell_seconds_))));

        publishState("RELEASING", tool_id_snapshot, tool_class_snapshot);
        publishGripperZeroer(true);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_zeroer_settle_seconds_))));

        if (!waitForGripperDone(gripper_done_timeout_seconds_)) {
            publishGripperZeroer(false);
            err = "timed out waiting for /gripper_done";
            return false;
        }
        publishGripperZeroer(false);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_open_pause_seconds_))));

        if (return_home_after_handover_) {
            std::string home_err;
            if (!doReturnHomeInternal(home_err)) {
                err = "return-home after handover failed: " + home_err;
                return false;
            }
        }
        return true;
    }

    // ── Skill: ReleaseTool ───────────────────────────────────────────

    bool doRelease(std::string &err) {
        (void)err;
        std::string tool_id_snapshot, tool_class_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            tool_id_snapshot = active_tool_id_;
            tool_class_snapshot = active_tool_class_;
        }
        publishState("RELEASING", tool_id_snapshot, tool_class_snapshot);
        publishGripper(true);  // open
        sleepForGripper();
        return true;
    }

    // ── Skill: ReturnHome ────────────────────────────────────────────

    bool doReturnHomeInternal(std::string &err) {
        publishState("RETURNING", "", "");
        if (home_joints_.size() != 6) {
            err = "home_joints must have 6 values";
            return false;
        }
        return moveToJointPositions(home_joints_, err);
    }

    // ── Action: PickTool ─────────────────────────────────────────────

    rclcpp_action::GoalResponse pickHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const PickTool::Goal> goal) {
        RCLCPP_INFO(get_logger(), "PickTool goal received: tool_id='%s'",
                    goal->tool_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse pickHandleCancel(
        const std::shared_ptr<GoalHandlePick>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void pickHandleAccepted(const std::shared_ptr<GoalHandlePick> goal_handle) {
        std::thread([this, goal_handle] { this->pickExecute(goal_handle); }).detach();
    }
    void pickExecute(const std::shared_ptr<GoalHandlePick> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<PickTool::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        const auto goal = goal_handle->get_goal();
        std::string err, picked_id, picked_class;
        const bool ok = doPick(goal->tool_id, picked_id, picked_class, err);
        result->success = ok;
        result->picked_tool_id = picked_id;
        result->picked_tool_class = picked_class;
        if (ok) {
            result->message = "ok";
            goal_handle->succeed(result);
        } else {
            result->message = err;
            publishState("IDLE", "", "");
            goal_handle->abort(result);
        }
    }

    // ── Action: HandoverTool ─────────────────────────────────────────

    rclcpp_action::GoalResponse handoverHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const HandoverTool::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handoverHandleCancel(
        const std::shared_ptr<GoalHandleHandover>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handoverHandleAccepted(
        const std::shared_ptr<GoalHandleHandover> goal_handle) {
        std::thread([this, goal_handle] { this->handoverExecute(goal_handle); }).detach();
    }
    void handoverExecute(const std::shared_ptr<GoalHandleHandover> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<HandoverTool::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        const auto goal = goal_handle->get_goal();
        std::string err;
        const bool ok = doHandover(goal->hand_pose, err);
        result->success = ok;
        result->message = ok ? "ok" : err;
        publishState("IDLE", "", "");
        if (ok) goal_handle->succeed(result);
        else    goal_handle->abort(result);
    }

    // ── Action: ReleaseTool ──────────────────────────────────────────

    rclcpp_action::GoalResponse releaseHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReleaseTool::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse releaseHandleCancel(
        const std::shared_ptr<GoalHandleRelease>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void releaseHandleAccepted(
        const std::shared_ptr<GoalHandleRelease> goal_handle) {
        std::thread([this, goal_handle] { this->releaseExecute(goal_handle); }).detach();
    }
    void releaseExecute(const std::shared_ptr<GoalHandleRelease> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<ReleaseTool::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        std::string err;
        const bool ok = doRelease(err);
        result->success = ok;
        result->message = ok ? "ok" : err;
        publishState("IDLE", "", "");
        if (ok) goal_handle->succeed(result);
        else    goal_handle->abort(result);
    }

    // ── Action: ReturnHome ───────────────────────────────────────────

    rclcpp_action::GoalResponse homeHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReturnHome::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse homeHandleCancel(
        const std::shared_ptr<GoalHandleHome>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void homeHandleAccepted(const std::shared_ptr<GoalHandleHome> goal_handle) {
        std::thread([this, goal_handle] { this->homeExecute(goal_handle); }).detach();
    }
    void homeExecute(const std::shared_ptr<GoalHandleHome> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<ReturnHome::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        std::string err;
        const bool ok = doReturnHomeInternal(err);
        result->success = ok;
        result->message = ok ? "ok" : err;
        publishState("IDLE", "", "");
        if (ok) goal_handle->succeed(result);
        else    goal_handle->abort(result);
    }

    // ── Members ──────────────────────────────────────────────────────

    // Params
    double z_offset_m_;
    double approach_height_m_;
    double tool_yaw_offset_rad_;
    std::string move_group_name_;
    std::string end_effector_link_;
    std::string reference_frame_;
    double velocity_scale_;
    double acceleration_scale_;
    double gripper_pause_seconds_;
    double handover_planning_time_;
    double handover_velocity_scale_;
    double handover_acceleration_scale_;
    double pre_release_dwell_seconds_;
    double post_zeroer_settle_seconds_;
    double post_open_pause_seconds_;
    bool return_home_after_handover_;
    double gripper_done_timeout_seconds_;
    double cartesian_min_fraction_;
    std::vector<std::string> joint_state_names_;
    geometry_msgs::msg::Point hand_offset_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    std::vector<double> home_joints_;

    // Pubs / subs
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<tracking_pkg::msg::HandState>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_;
    rclcpp::Client<tracking_pkg::srv::GetWorldState>::SharedPtr world_state_client_;

    // Action servers
    rclcpp_action::Server<PickTool>::SharedPtr pick_srv_;
    rclcpp_action::Server<HandoverTool>::SharedPtr handover_srv_;
    rclcpp_action::Server<ReleaseTool>::SharedPtr release_srv_;
    rclcpp_action::Server<ReturnHome>::SharedPtr home_srv_;

    // MoveIt
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // State
    std::mutex execution_mutex_;
    std::mutex state_mutex_;
    std::mutex gripper_done_mutex_;
    std::condition_variable gripper_done_cv_;
    bool gripper_done_received_ = false;
    tracking_pkg::msg::HandState last_hand_state_;
    bool have_hand_state_ = false;
    std::string current_state_ = "IDLE";
    std::string active_tool_id_;
    std::string active_tool_class_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SkillExecutor>();
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
