// Skill Executor Node
// ===================
// Provides four ROS actions that wrap the MoveIt-based pick / handover /
// release / return_home skills from tool_pick_test_node.cpp, plus publishes
// /system_state_update so world_model_node can surface the current state.
//
// Actions:
//   /pick_tool       (tracking_msgs/action/PickTool)
//   /grasp_tool      (tracking_msgs/action/GraspTool)
//   /handover_tool   (tracking_msgs/action/HandoverTool)
//   /release_tool    (tracking_msgs/action/ReleaseTool)
//   /return_home     (tracking_msgs/action/ReturnHome)
//   /return_tool     (tracking_msgs/action/ReturnTool)
//
// State updates on /system_state_update use the convention
//   "STATE:tool_id:tool_class" parsed by world_model_node._state_cb.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// MoveGroupInterface::Plan member is `trajectory_` on Humble but `trajectory` on
// Jazzy (the same release that migrated MoveIt headers .h -> .hpp). Key off that
// header migration so this builds on the NUC (Humble) and the Spark (Jazzy).
#if __has_include(<moveit/version.hpp>)
#  define RSN_PLAN_TRAJECTORY trajectory
#else
#  define RSN_PLAN_TRAJECTORY trajectory_
#endif
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <tracking_msgs/msg/grasp_candidate.hpp>
#include <tracking_msgs/msg/hand_state.hpp>
#include <tracking_msgs/msg/tool_event.hpp>
#include <tracking_msgs/srv/get_tool_home.hpp>
#include <tracking_msgs/srv/get_world_state.hpp>
#include <tracking_msgs/action/pick_tool.hpp>
#include <tracking_msgs/action/grasp_tool.hpp>
#include <tracking_msgs/action/handover_tool.hpp>
#include <tracking_msgs/action/release_tool.hpp>
#include <tracking_msgs/action/return_home.hpp>
#include <tracking_msgs/action/return_tool.hpp>
#include <tracking_msgs/action/return_tool_home.hpp>

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

using PickTool = tracking_msgs::action::PickTool;
using GraspTool = tracking_msgs::action::GraspTool;
using HandoverTool = tracking_msgs::action::HandoverTool;
using ReleaseTool = tracking_msgs::action::ReleaseTool;
using ReturnHome = tracking_msgs::action::ReturnHome;
using ReturnTool = tracking_msgs::action::ReturnTool;
using ReturnToolHome = tracking_msgs::action::ReturnToolHome;

using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickTool>;
using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<GraspTool>;
using GoalHandleHandover = rclcpp_action::ServerGoalHandle<HandoverTool>;
using GoalHandleRelease = rclcpp_action::ServerGoalHandle<ReleaseTool>;
using GoalHandleHome = rclcpp_action::ServerGoalHandle<ReturnHome>;
using GoalHandleReturnTool = rclcpp_action::ServerGoalHandle<ReturnTool>;
using GoalHandleReturnHome_ = rclcpp_action::ServerGoalHandle<ReturnToolHome>;

// Tray names, as stamped onto every GraspCandidate by tool_detection_node's
// `location` parameter. GraspTool defaults to the reclaim tray: picking from the
// instrument tray goes through PickTool, which also presents the tool.
constexpr const char *kInstrumentLocation = "instrument_tray";
constexpr const char *kReclaimLocation = "reclaim_tray";

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

bool hasUsableWorldXy(const tracking_msgs::msg::GraspCandidate &c) {
    if (!isFinitePoint(c.grasp_pose.pose.position)) return false;
    if (c.grasp_pose.header.frame_id != "world") return false;
    return std::hypot(c.grasp_pose.pose.position.x, c.grasp_pose.pose.position.y) > 1e-4;
}

moveit_msgs::msg::RobotState makeStartStateFromPlanEnd(
    const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    moveit_msgs::msg::RobotState rs;
    rs.is_diff = false;
    rs.joint_state.name = plan.RSN_PLAN_TRAJECTORY.joint_trajectory.joint_names;
    if (!plan.RSN_PLAN_TRAJECTORY.joint_trajectory.points.empty()) {
        rs.joint_state.position = plan.RSN_PLAN_TRAJECTORY.joint_trajectory.points.back().positions;
    }
    return rs;
}

}  // namespace


class SkillExecutor : public rclcpp::Node {
public:
    SkillExecutor()
    : Node("skill_executor_node") {
        // ── Parameters (same defaults as tool_pick_test_node.cpp) ──────
        z_offset_m_ = declare_parameter("z_offset", 0.004);
        // Same thing for the reclaim tray, kept separate: its grasp points come
        // from a side camera projected onto a fixed plane, so its depth error is
        // not the instrument tray's. 5 mm shallower than the instrument tray.
        reclaim_z_offset_m_ = declare_parameter("reclaim_z_offset", 0.009);
        approach_height_m_ = declare_parameter("approach_height_m", 0.04);
        // Fast local re-grasp: on a failed/slipped grasp, retry at the tray with
        // an escalating pose nudge instead of present→home→re-perceive.
        max_regrasp_retries_ = declare_parameter("max_regrasp_retries", 2);
        regrasp_deeper_step_m_ = declare_parameter("regrasp_deeper_step_m", 0.0015);
        regrasp_center_step_m_ = declare_parameter("regrasp_center_step_m", 0.005);
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
        // Max wait for the robotiq grasp-result (/tool_grasped) after closing.
        grasp_check_timeout_sec_ = declare_parameter("grasp_check_timeout_sec", 5.0);
        // Settle time after the lift before re-verifying the grasp (a marginal
        // grip can relax in the first moment after lifting).
        post_lift_settle_sec_ = declare_parameter("post_lift_settle_sec", 0.5);
        cartesian_min_fraction_ = declare_parameter("cartesian_min_fraction", 0.95);
        // Handover waits for the surgeon's double_open_close gesture before
        // moving to the hand. 0.0 = wait indefinitely.
        gesture_wait_timeout_sec_ = declare_parameter("gesture_wait_timeout_sec", 0.0);
        post_gesture_settle_sec_ = declare_parameter("post_gesture_settle_sec", 0.5);
        // return_tool releases the wrong tool this far above the pickup pose.
        return_release_height_m_ = declare_parameter("return_release_height_m", 0.005);
        // grasp_tool on the reclaim tray rises to this absolute world z after the
        // lift, before attaching the held-tool box. The reclaim tray sits inside
        // a bracket whose top bar spans z ~= [-0.015, +0.015]; attaching the
        // 0.30 m tool box down at the approach pose would put it through the
        // bracket's drop wall and wedge every later plan.
        reclaim_hold_z_m_ = declare_parameter("reclaim_hold_z_m", 0.10);
        // After a successful pick the arm rotates shoulder_pan to this angle to
        // present the tool. 250 deg = 4.36332 rad (requires the widened
        // shoulder_pan limit on the live robot).
        present_shoulder_pan_rad_ = declare_parameter("present_shoulder_pan_rad", 4.36332313);
        // Wrist angles for the presentation/handover tool orientation, applied
        // as a joint-space move (no IK) after the pan rotation so the tool is
        // already oriented before the move to the hand. Tune by jogging the arm
        // to the desired turned-around handover orientation and reading
        // wrist_2_joint / wrist_3_joint. Defaults = home-pose wrist values.
        present_wrist1_rad_ = declare_parameter("present_wrist1_rad", -1.5248240244);
        present_wrist2_rad_ = declare_parameter("present_wrist2_rad", -1.2305892150);
        present_wrist3_rad_ = declare_parameter("present_wrist3_rad", -4.8166621367);

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
        // Requests an on-demand fresh grasp re-check (e.g. right after a lift,
        // where the async loss monitor may still be lagging).
        verify_grasp_pub_ = create_publisher<std_msgs::msg::Empty>("/verify_grasp", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/system_state_update", 10);
        // The transitions perception cannot see. /system_state_update is no
        // substitute: its RELEASING state is published by three different paths,
        // and the moment the surgeon actually takes the tool (the force-guided
        // /gripper_done) is invisible from outside. An instrument count must not
        // rest on that inference. See ToolEvent.msg.
        tool_event_pub_ = create_publisher<tracking_msgs::msg::ToolEvent>(
            "/tool_event", 10);
        tool_home_client_ = create_client<tracking_msgs::srv::GetToolHome>(
            "/get_tool_home");
        // Latched so a late-joining hand_tracker picks up the current value.
        // True only while a handover is actively waiting for the gesture; the
        // hand_tracker gates /hand_gesture publication on this flag.
        handover_waiting_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/handover_waiting",
            rclcpp::QoS(1).transient_local());
        publishHandoverWaiting(false);

        // ── Subscribers ──────────────────────────────────────────────
        hand_state_sub_ = create_subscription<tracking_msgs::msg::HandState>(
            "/hand_state", 10,
            std::bind(&SkillExecutor::handStateCb, this, std::placeholders::_1));
        gesture_sub_ = create_subscription<std_msgs::msg::String>(
            "/hand_gesture", 10,
            std::bind(&SkillExecutor::gestureCb, this, std::placeholders::_1));
        gripper_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/gripper_done", 10,
            std::bind(&SkillExecutor::gripperDoneCb, this, std::placeholders::_1));
        tool_grasped_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/tool_grasped", 10,
            std::bind(&SkillExecutor::toolGraspedCb, this, std::placeholders::_1));

        // ── Service client ───────────────────────────────────────────
        world_state_client_ =
            create_client<tracking_msgs::srv::GetWorldState>("/get_world_state");

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

        grasp_srv_ = rclcpp_action::create_server<GraspTool>(
            this, "grasp_tool",
            std::bind(&SkillExecutor::graspHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::graspHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::graspHandleAccepted, this,
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

        return_home_slot_srv_ = rclcpp_action::create_server<ReturnToolHome>(
            this, "return_tool_home",
            std::bind(&SkillExecutor::returnHomeSlotHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::returnHomeSlotHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::returnHomeSlotHandleAccepted, this,
                      std::placeholders::_1));

        return_srv_ = rclcpp_action::create_server<ReturnTool>(
            this, "return_tool",
            std::bind(&SkillExecutor::returnToolHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SkillExecutor::returnToolHandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SkillExecutor::returnToolHandleAccepted, this,
                      std::placeholders::_1));

        publishState("IDLE", "", "");
        RCLCPP_INFO(get_logger(), "SkillExecutor action servers ready.");
    }

private:
    // ── Callbacks ────────────────────────────────────────────────────

    void handStateCb(const tracking_msgs::msg::HandState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_hand_state_ = *msg;
        have_hand_state_ = true;
    }

    void gestureCb(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != "double_open_close") return;
        {
            std::lock_guard<std::mutex> lock(gesture_mutex_);
            gesture_received_ = true;
        }
        gesture_cv_.notify_all();
    }

    void gripperDoneCb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        {
            std::lock_guard<std::mutex> lock(gripper_done_mutex_);
            gripper_done_received_ = true;
        }
        gripper_done_cv_.notify_all();
    }

    // Robotiq grasp result: true = a tool is held, false = gripper empty.
    // Published once after each close and continuously by the loss monitor.
    void toolGraspedCb(const std_msgs::msg::Bool::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(tool_grasped_mutex_);
            tool_grasped_value_ = msg->data;
            tool_grasped_received_ = true;
            tool_grasped_ever_ = true;
        }
        tool_grasped_cv_.notify_all();
        // React immediately if the tool is lost mid-transport: stop the running
        // motion instead of finishing the (now pointless) present moves.
        if (!msg->data && transporting_.load()) {
            RCLCPP_WARN(get_logger(),
                "Tool lost during transport — stopping motion.");
            move_group_->stop();
        }
    }

    // ── Tool events ──────────────────────────────────────────────────
    // What actually happened to a specific tool. The registry keeps the
    // instrument count on these, so they must be truthful: HANDED_OVER is
    // published only once the surgeon has physically pulled the tool out of the
    // jaws, not when we merely intended to give it to him.

    void publishToolEvent(const std::string &event,
                          const std::string &track_id,
                          const std::string &tool_class,
                          const std::string &from_location,
                          const geometry_msgs::msg::Point &handle_center,
                          const geometry_msgs::msg::Vector3 &functional_end_dir,
                          double grasp_distance_m) {
        tracking_msgs::msg::ToolEvent msg;
        msg.header.stamp = now();
        msg.header.frame_id = reference_frame_;
        msg.event = event;
        msg.track_id = track_id;
        msg.tool_class = tool_class;
        msg.from_location = from_location;
        msg.handle_center = handle_center;
        msg.functional_end_dir = functional_end_dir;
        msg.grasp_distance_m = static_cast<float>(grasp_distance_m);
        tool_event_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "ToolEvent %s: %s (%s)",
                    event.c_str(), track_id.c_str(), tool_class.c_str());
    }

    // The tool currently in the gripper, remembered so an event fired later (a
    // handover completing, a drop) can still name it. Cleared when it leaves.
    void rememberHeldTool(const tracking_msgs::msg::GraspCandidate &cand,
                          const geometry_msgs::msg::Pose &grasp_pose) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        held_track_id_ = cand.tool_id;
        held_tool_class_ = cand.tool_class;
        held_from_location_ = cand.location;
        held_handle_center_ = cand.handle_center;
        held_end_dir_ = cand.functional_end_dir;
        // Where along the tool the jaws hold it. This — not the grasp pose — is
        // what lets the tool be put back at the right spot, because the grasp
        // distance differs per tray (the sliding strategy uses that tray's hole).
        held_grasp_distance_m_ = std::hypot(
            grasp_pose.position.x - cand.handle_center.x,
            grasp_pose.position.y - cand.handle_center.y);
        have_held_tool_ = true;
    }

    void publishHeldToolEvent(const std::string &event) {
        std::string track, cls, loc;
        geometry_msgs::msg::Point hc;
        geometry_msgs::msg::Vector3 dir;
        double d = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_held_tool_) return;
            track = held_track_id_;
            cls = held_tool_class_;
            loc = held_from_location_;
            hc = held_handle_center_;
            dir = held_end_dir_;
            d = held_grasp_distance_m_;
        }
        publishToolEvent(event, track, cls, loc, hc, dir, d);
        if (event != "PICKED") {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_held_tool_ = false;   // it is out of the gripper now
        }
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
                || state == "HANDOVER" || state == "RELEASING"
                || state == "HOLDING") {
                active_tool_id_ = tool_id;
                active_tool_class_ = tool_class;
            } else if (state == "IDLE") {
                active_tool_id_ = "";
                active_tool_class_ = "";
            }
        }
    }

    // ── World-state lookup ───────────────────────────────────────────

    bool fetchCandidates(std::vector<tracking_msgs::msg::GraspCandidate> &out,
                         std::string &err) {
        if (!world_state_client_->wait_for_service(std::chrono::seconds(2))) {
            err = "/get_world_state service not available";
            return false;
        }
        auto req = std::make_shared<tracking_msgs::srv::GetWorldState::Request>();
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

    // location_filter: "instrument_tray" | "reclaim_tray", or "" for both trays.
    // The world model returns candidates from both trays in one list; each one
    // carries the tray it was seen on in `location`.
    bool collectCandidates(std::vector<tracking_msgs::msg::GraspCandidate> &out_sorted,
                           const std::string &location_filter,
                           std::string &err) {
        std::vector<tracking_msgs::msg::GraspCandidate> all;
        if (!fetchCandidates(all, err)) return false;
        for (const auto &c : all) {
            if (!hasUsableWorldXy(c)) continue;
            if (!location_filter.empty() && c.location != location_filter) continue;
            out_sorted.push_back(c);
        }
        std::sort(out_sorted.begin(), out_sorted.end(),
                  [](const auto &a, const auto &b) {
                      return a.grasp_confidence > b.grasp_confidence;
                  });
        if (out_sorted.empty()) {
            err = "no usable candidates from world model" +
                  (location_filter.empty() ? std::string()
                                           : (" at " + location_filter));
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
        plan.RSN_PLAN_TRAJECTORY = traj;
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
        plan_out.RSN_PLAN_TRAJECTORY = traj;
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

    // Rotate shoulder_pan to a target angle, holding all other joints at their
    // current values. Uses a joint-space goal (not a pose target) so the path
    // stays deterministic instead of wandering through random IK solutions.
    bool rotateShoulderPanTo(double target_rad, std::string &err) {
        std::vector<double> joints = move_group_->getCurrentJointValues();
        if (joints.size() != joint_state_names_.size()) {
            err = "could not read current joint values";
            return false;
        }
        // shoulder_pan_joint is the first entry of joint_state_names_.
        auto it = std::find(joint_state_names_.begin(), joint_state_names_.end(),
                            std::string("shoulder_pan_joint"));
        if (it == joint_state_names_.end()) {
            err = "shoulder_pan_joint not in joint_state_names";
            return false;
        }
        joints[std::distance(joint_state_names_.begin(), it)] = target_rad;
        return moveToJointPositions(joints, err);
    }

    // Rotate wrist_1, wrist_2 and wrist_3 to their presentation/handover angles,
    // holding every other joint at its current value. Joint-space (no IK), so it
    // cannot fail to "sample valid goal states" the way a pose target can.
    bool preorientWrists(std::string &err) {
        std::vector<double> joints = move_group_->getCurrentJointValues();
        if (joints.size() != joint_state_names_.size()) {
            err = "could not read current joint values";
            return false;
        }
        const auto set_joint = [&](const std::string &name, double val) {
            auto it = std::find(joint_state_names_.begin(),
                                joint_state_names_.end(), name);
            if (it == joint_state_names_.end()) return false;
            joints[std::distance(joint_state_names_.begin(), it)] = val;
            return true;
        };
        if (!set_joint("wrist_1_joint", present_wrist1_rad_) ||
            !set_joint("wrist_2_joint", present_wrist2_rad_) ||
            !set_joint("wrist_3_joint", present_wrist3_rad_)) {
            err = "wrist_1/wrist_2/wrist_3 not in joint_state_names";
            return false;
        }
        return moveToJointPositions(joints, err);
    }

    // ── Attached tool collision object ───────────────────────────────
    // A single conservative box for every tool: 30 cm along the TCP Y axis,
    // 5 cm in X and Z, centered on gripper_tip_link (tool gripped in the
    // middle). Attaching it makes the held tool visible to the planner so it
    // avoids the tray-camera stand during the rotation and handover, and
    // explains the real reach of the gripper beyond its own collision box.

    void attachToolBox() {
        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = end_effector_link_;
        aco.object.id = "held_tool";
        aco.object.header.frame_id = end_effector_link_;
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.05, 0.30, 0.05};  // x, y (long axis), z

        geometry_msgs::msg::Pose pose;  // identity = centered on the TCP
        pose.orientation.w = 1.0;

        aco.object.primitives.push_back(box);
        aco.object.primitive_poses.push_back(pose);
        // Links the box is allowed to touch (the gripper body it is held by).
        aco.touch_links = {
            "gripper_tip_link", "dummy_gripper_link", "wrist_3_link",
            "flange", "tool0",
        };
        psi_.applyAttachedCollisionObject(aco);
        RCLCPP_INFO(get_logger(), "Attached held_tool collision box to %s.",
                    end_effector_link_.c_str());
    }

    void detachToolBox() {
        // Detach from the gripper and remove from the scene. Idempotent: if no
        // tool is attached this is a harmless no-op.
        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = end_effector_link_;
        aco.object.id = "held_tool";
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        psi_.applyAttachedCollisionObject(aco);
        psi_.removeCollisionObjects({"held_tool"});
    }

    // Tool was lost while we believed we were holding it: drop the planner's
    // attached box, open the gripper, return home so the tray camera is free,
    // and set a tool_lost error the action surfaces to the orchestrator.
    void abortHoldingAndGoHome(const std::string &reason, std::string &err) {
        RCLCPP_WARN(get_logger(),
            "Tool lost (%s). Detaching, releasing and returning home.",
            reason.c_str());
        detachToolBox();
        publishGripper(true);   // open
        // The tool fell somewhere we did not intend. The registry must not keep
        // believing the robot holds it — mark it unknown and let perception find
        // it again if it landed on a tray.
        publishHeldToolEvent("DROPPED");
        std::string home_err;
        doReturnHomeInternal(home_err);
        err = "tool_lost: " + reason;
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

    void publishHandoverWaiting(bool waiting) {
        std_msgs::msg::Bool m;
        m.data = waiting;
        handover_waiting_pub_->publish(m);
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

    // Waits indefinitely for the surgeon to take the tool (force-triggered
    // /gripper_done). Unlike waitForGripperDone there is no time limit — the
    // robot holds the tool out for as long as it takes. Still wakes every 250 ms
    // to stay responsive to goal cancellation (abort) and return_tool
    // preemption, so a stuck handover can be broken. Returns true on
    // /gripper_done, false on cancel/preempt/shutdown.
    bool waitForGripperDoneIndefinite(
            const std::shared_ptr<GoalHandleHandover> &goal_handle) {
        std::unique_lock<std::mutex> lock(gripper_done_mutex_);
        gripper_done_received_ = false;
        while (rclcpp::ok()) {
            if (gripper_done_cv_.wait_for(lock, std::chrono::milliseconds(250),
                    [this] { return gripper_done_received_; })) {
                return true;
            }
            if (goal_handle->is_canceling()) return false;
            {
                std::lock_guard<std::mutex> g(gesture_mutex_);
                if (abort_handover_) return false;
            }
        }
        return false;
    }

    // Waits for a fresh /tool_grasped after a close command and returns whether
    // a tool is held. A timeout counts as "not grasped".
    bool waitForFreshToolGrasped(double timeout_s) {
        std::unique_lock<std::mutex> lock(tool_grasped_mutex_);
        tool_grasped_received_ = false;
        const bool ok = tool_grasped_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(timeout_s)),
            [this] { return tool_grasped_received_; });
        if (!ok) return false;
        return tool_grasped_value_;
    }

    // Latest known grasp state without waiting. have_data is false until the
    // first /tool_grasped message arrives (so callers don't act on a default).
    bool lastToolGrasped(bool &have_data) {
        std::lock_guard<std::mutex> lock(tool_grasped_mutex_);
        have_data = tool_grasped_ever_;
        return tool_grasped_value_;
    }

    // Forces a fresh grasp reading after a lift. The async loss monitor only
    // polls every ~0.5 s, so lastToolGrasped() can still report "held" right
    // after a lift even though the tool already slipped. Triggering an explicit
    // re-check via /verify_grasp gives a deterministic answer before we commit
    // to the (expensive) present rotation. A timed-out re-check assumes the tool
    // is still held (the close-time check already confirmed the grasp).
    bool verifyGraspAfterLift() {
        std::unique_lock<std::mutex> lock(tool_grasped_mutex_);
        tool_grasped_received_ = false;
        lock.unlock();
        verify_grasp_pub_->publish(std_msgs::msg::Empty());
        lock.lock();
        const bool ok = tool_grasped_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(grasp_check_timeout_sec_)),
            [this] { return tool_grasped_received_; });
        if (!ok) {
            RCLCPP_WARN(get_logger(),
                "Post-lift grasp re-verify timed out; assuming still held.");
            return true;
        }
        return tool_grasped_value_;
    }

    // Fresh check of whether the gripper currently holds a tool, used as a guard
    // before a new pick. Forces a /verify_grasp query so the answer reflects the
    // real gOBJ now (not a possibly-stale monitor value). A timeout assumes NOT
    // holding, so a gripper-comms hiccup never blocks all picks.
    bool freshGripperHoldsTool() {
        std::unique_lock<std::mutex> lock(tool_grasped_mutex_);
        tool_grasped_received_ = false;
        lock.unlock();
        verify_grasp_pub_->publish(std_msgs::msg::Empty());
        lock.lock();
        const bool ok = tool_grasped_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(grasp_check_timeout_sec_)),
            [this] { return tool_grasped_received_; });
        if (!ok) {
            RCLCPP_WARN(get_logger(),
                "Holding-guard re-check timed out; assuming gripper empty.");
            return false;
        }
        return tool_grasped_value_;
    }

    // Blocks until a fresh double_open_close gesture arrives. Any gesture
    // seen before this call is discarded so a gesture made while the tool
    // was being picked does not count. Wakes every 250 ms to check whether
    // the goal is being cancelled or return_tool is preempting the handover,
    // so both stay responsive. gesture_wait_timeout_sec_ <= 0 means wait
    // indefinitely. Returns true only on a real gesture (false on cancel,
    // preemption or timeout).
    bool waitForGesture(const std::shared_ptr<GoalHandleHandover> &goal_handle) {
        std::unique_lock<std::mutex> lock(gesture_mutex_);
        gesture_received_ = false;
        const auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            if (gesture_cv_.wait_for(lock, std::chrono::milliseconds(250),
                    [this] { return gesture_received_ || abort_handover_; })) {
                return !abort_handover_;  // true only on a real gesture
            }
            if (goal_handle->is_canceling()) return false;
            // Don't keep waiting to hand over a tool that fell out meanwhile.
            bool have_grasp_data = false;
            if (!lastToolGrasped(have_grasp_data) && have_grasp_data) return false;
            if (gesture_wait_timeout_sec_ > 0.0) {
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed >= gesture_wait_timeout_sec_) return false;
            }
        }
        return false;
    }

    void publishHandoverFeedback(
        const std::shared_ptr<GoalHandleHandover> &goal_handle,
        const std::string &phase) {
        auto fb = std::make_shared<HandoverTool::Feedback>();
        fb->phase = phase;
        goal_handle->publish_feedback(fb);
    }

    // ── Pick sequence planning ───────────────────────────────────────

    // z_offset_m: how far ABOVE the perceived grasp point the TCP stops. Per-tray,
    // because the two trays are perceived differently (top-down camera + depth-free
    // plane on the instrument tray vs. a side camera on the reclaim tray), so their
    // depth errors do not have the same sign or size.
    bool tryPlanPickSequence(
        const tracking_msgs::msg::GraspCandidate &cand,
        double z_offset_m,
        moveit::planning_interface::MoveGroupInterface::Plan &approach_plan,
        moveit::planning_interface::MoveGroupInterface::Plan &descend_plan,
        moveit::planning_interface::MoveGroupInterface::Plan &lift_plan,
        geometry_msgs::msg::Pose &approach_pose_out,
        geometry_msgs::msg::Pose &grasp_pose_out,
        std::string &err) {
        grasp_pose_out.position.x = cand.grasp_pose.pose.position.x;
        grasp_pose_out.position.y = cand.grasp_pose.pose.position.y;
        // Three terms, each with one job:
        //   grasp_pose.z    the tray plane (perception)
        //   grasp_z_offset  per-class strategy: tall tools shallower, thin tools
        //                   deeper — but only ever negative when grasp_geometry
        //                   confirmed the point is over a measured tray opening
        //   z_offset_m      the global per-tray trim
        grasp_pose_out.position.z = cand.grasp_pose.pose.position.z
                                  + cand.grasp_z_offset
                                  + z_offset_m;
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

    // ── Grasp core (shared by PickTool and GraspTool) ────────────────

    // Holding-guard → candidate selection → pre-flight (approach + descend +
    // lift all planned before any motion) → approach → open → descend → close →
    // /tool_grasped check → lift → post-lift verify → escalating local re-grasp
    // → attachToolBox().
    //
    // Ends with the tool in the gripper and moves the arm no further. Presenting,
    // handing over and placing are the caller's business.
    //
    // `clearout_world_z` is an absolute world z to rise to after the lift and
    // before the tool box is attached; pass NaN to skip it (see below).
    // `chosen_out` is filled as soon as pre-flight commits to a candidate, so a
    // caller can name the tool even when a later motion phase fails.
    bool graspToolCore(const std::string &tool_id_arg,
                       const std::string &location_filter,
                       double clearout_world_z,
                       tracking_msgs::msg::GraspCandidate &chosen_out,
                       geometry_msgs::msg::Pose &grasp_pose_out,
                       geometry_msgs::msg::Pose &approach_pose_out,
                       std::string &err) {
        // Holding-guard: never start a grasp while the gripper already holds a
        // tool — the approach phase opens the gripper and would drop it. A fresh
        // check (not a possibly-stale monitor value) reflects the real state.
        if (freshGripperHoldsTool()) {
            std::string held_class;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                held_class = active_tool_class_;
            }
            err = "already_holding_tool" +
                  (held_class.empty() ? std::string() : (": " + held_class));
            RCLCPP_WARN(get_logger(),
                "Grasp refused: gripper already holds a tool (%s). "
                "Hand it over or return it first.",
                held_class.empty() ? "unknown" : held_class.c_str());
            return false;
        }

        std::vector<tracking_msgs::msg::GraspCandidate> candidates;
        if (!collectCandidates(candidates, location_filter, err)) return false;

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

        const double z_off = (location_filter == kReclaimLocation)
            ? reclaim_z_offset_m_ : z_offset_m_;

        // Pre-flight: try each candidate in confidence order. Only when
        // approach + descend + lift all plan successfully do we commit to
        // a grasp. No motion happens before this loop succeeds.
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan, descend_plan, lift_plan;
        geometry_msgs::msg::Pose approach_pose, grasp_pose;
        tracking_msgs::msg::GraspCandidate chosen;
        std::vector<std::string> rejection_log;
        bool found = false;
        for (const auto &cand : candidates) {
            std::string plan_err;
            if (tryPlanPickSequence(cand, z_off, approach_plan, descend_plan, lift_plan,
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

        chosen_out = chosen;
        grasp_pose_out = grasp_pose;
        approach_pose_out = approach_pose;

        RCLCPP_INFO(get_logger(),
            "Pre-flight OK for %s (%s). grasp z=%.4f approach z=%.4f. Executing.",
            chosen.tool_id.c_str(), chosen.tool_class.c_str(),
            grasp_pose.position.z, approach_pose.position.z);

        publishState("PICKING", chosen.tool_id, chosen.tool_class);

        if (!executePlan(approach_plan, err)) { err = "approach exec: " + err; return false; }
        publishGripper(true);   // open
        sleepForGripper();

        // Two failure modes are handled differently:
        //  - gripper closes on nothing (empty at close): the tool isn't where we
        //    expected -> go home and let the LLM re-perceive and retry. We do
        //    NOT keep stabbing at the tray blindly.
        //  - tool grasped but slips while lifting: the grasp was just marginal
        //    -> re-grasp locally with an escalating pose nudge (deeper + toward
        //    the tool centre along functional_end_dir), up to
        //    max_regrasp_retries_ times, without the slow present→home cycle.
        bool secured = false;
        for (int attempt = 0; attempt <= max_regrasp_retries_ && !secured; ++attempt) {
            moveit::planning_interface::MoveGroupInterface::Plan descend_try = descend_plan;
            moveit::planning_interface::MoveGroupInterface::Plan lift_try = lift_plan;

            if (attempt > 0) {
                geometry_msgs::msg::Pose grasp_try = grasp_pose;
                grasp_try.position.z -= attempt * regrasp_deeper_step_m_;
                const double fx = chosen.functional_end_dir.x;
                const double fy = chosen.functional_end_dir.y;
                const double fn = std::hypot(fx, fy);
                if (fn > 1e-6) {
                    grasp_try.position.x += attempt * regrasp_center_step_m_ * fx / fn;
                    grasp_try.position.y += attempt * regrasp_center_step_m_ * fy / fn;
                }
                RCLCPP_INFO(get_logger(),
                    "Re-grasp attempt %d for %s: %.1f mm deeper, %.1f mm toward center.",
                    attempt, chosen.tool_id.c_str(),
                    attempt * regrasp_deeper_step_m_ * 1000.0,
                    attempt * regrasp_center_step_m_ * 1000.0);

                std::string perr;
                move_group_->setStartStateToCurrentState();
                if (!planLinearPose(grasp_try, descend_try, perr)) {
                    RCLCPP_WARN(get_logger(), "Re-grasp descend plan failed: %s", perr.c_str());
                    continue;
                }
                move_group_->setStartState(makeStartStateFromPlanEnd(descend_try));
                if (!planLinearPose(approach_pose, lift_try, perr)) {
                    RCLCPP_WARN(get_logger(), "Re-grasp lift plan failed: %s", perr.c_str());
                    continue;
                }
            }

            if (!executePlan(descend_try, err)) { err = "descend exec: " + err; return false; }
            publishGripper(false);  // close
            const bool grasped = waitForFreshToolGrasped(grasp_check_timeout_sec_);

            if (!grasped) {
                // Empty at close -> re-perceive instead of re-grasping blindly.
                RCLCPP_WARN(get_logger(),
                    "Grasp check failed (gripper empty at close). Returning home.");
                publishGripper(true);
                std::string lift_err;
                executePlan(lift_try, lift_err);   // raise empty gripper off the tray
                std::string home_err;
                doReturnHomeInternal(home_err);
                err = "grasp_failed: no tool in gripper after close";
                return false;
            }

            // Grasped -> lift, briefly settle, then force a fresh check. The
            // settle lets a marginal grip relax before we commit, and the fresh
            // check avoids the async monitor's ~0.5 s lag.
            if (!executePlan(lift_try, err)) { err = "lift exec: " + err; return false; }
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.0, post_lift_settle_sec_))));
            if (verifyGraspAfterLift()) {
                secured = true;
                break;
            }

            // Slipped during the lift -> open and re-grasp locally with a bigger
            // nudge. No present rotation, no home.
            RCLCPP_WARN(get_logger(),
                "Tool slipped during lift on attempt %d. Re-grasping.", attempt);
            publishGripper(true);
            sleepForGripper();
        }

        if (!secured) {
            publishGripper(true);
            std::string home_err;
            doReturnHomeInternal(home_err);   // best-effort: free the tray camera
            err = "grasp_failed: tool slips during lift after " +
                  std::to_string(max_regrasp_retries_ + 1) + " attempts";
            return false;
        }

        // Clear-out lift, before the tool box is attached.
        //
        // attachToolBox() hangs a 0.30 m long box off the TCP. On the reclaim
        // tray the approach height is still down inside the tray: the box would
        // sweep through the tray's vertical drop wall, and MoveIt would then
        // refuse every later plan with "start state in collision" — the arm
        // would be stuck holding the tool. So rise clear of the tray first and
        // attach up there. The instrument tray needs none of this (its approach
        // pose is already in free space), so doPick passes NaN and this is
        // skipped — its behaviour is unchanged.
        if (std::isfinite(clearout_world_z) &&
            clearout_world_z > approach_pose.position.z + 1e-6) {
            geometry_msgs::msg::Pose clear = approach_pose;
            clear.position.z = clearout_world_z;
            moveit::planning_interface::MoveGroupInterface::Plan clear_plan;
            move_group_->setStartStateToCurrentState();
            std::string clear_err;
            if (planLinearPose(clear, clear_plan, clear_err) &&
                executePlan(clear_plan, clear_err)) {
                RCLCPP_INFO(get_logger(),
                    "Clear-out lift to z=%.3f before attaching tool box.",
                    clearout_world_z);
            } else if (!moveToPoseTarget(clear, clear_err)) {
                // Do not attach the box at a pose where it is in collision —
                // that would wedge the planner. Bail out holding the tool and
                // let the caller decide.
                err = "clear-out lift: " + clear_err;
                return false;
            }
        }

        // Tool is now in the gripper: make it visible to the planner so any
        // onward motion (present rotation, handover, place-back) avoids the
        // tray-camera stand and accounts for the tool's reach beyond the box.
        attachToolBox();

        rememberHeldTool(chosen, grasp_pose);
        publishHeldToolEvent("PICKED");
        return true;
    }

    // ── Skill: PickTool ──────────────────────────────────────────────

    bool doPick(const std::string &tool_id_arg,
                std::string &picked_id_out,
                std::string &picked_class_out,
                std::string &err) {
        tracking_msgs::msg::GraspCandidate chosen;
        geometry_msgs::msg::Pose grasp_pose, approach_pose;

        // A tool on the reclaim tray is a USED tool. Exactly one thing may happen to
        // it: go back to the instrument tray (return_tool_home). It is never handed
        // to the surgeon. So a pick that leads to a handover only ever sources from
        // the instrument tray — asking for a reclaim id here fails with
        // "tool_id 'reclaim_N' not in candidates", which is the right answer.
        //
        // NaN = no clear-out lift: the instrument tray's approach pose is already in
        // free space, so the tool box can be attached right there.
        const bool grasped = graspToolCore(
            tool_id_arg, kInstrumentLocation,
            std::numeric_limits<double>::quiet_NaN(),
            chosen, grasp_pose, approach_pose, err);
        // chosen is filled the moment pre-flight commits, so the tool can still
        // be named when a later motion phase fails — the LLM keys its retry on
        // the class.
        picked_id_out = chosen.tool_id;
        picked_class_out = chosen.tool_class;
        if (!grasped) return false;

        publishState("TRANSPORTING", chosen.tool_id, chosen.tool_class);
        // From here the tool is in transit: a /tool_grasped=false stops the
        // current motion immediately (see toolGraspedCb). After each move we
        // check whether that happened and bail to home if so.
        transporting_.store(true);

        // Immediately turn the arm around to present the tool. The attached
        // tool box + the tray-camera stand in the scene keep this collision-free.
        const bool present_ok = rotateShoulderPanTo(present_shoulder_pan_rad_, err);
        {
            bool hd = false;
            if (!lastToolGrasped(hd) && hd) {
                transporting_.store(false);
                abortHoldingAndGoHome("tool dropped during present rotation", err);
                return false;
            }
        }
        if (!present_ok) {
            transporting_.store(false);
            err = "present rotation: " + err;
            return false;
        }

        // Pre-orient the wrist into the handover tool orientation now (while
        // turned away), as a joint-space move on wrist_2/wrist_3 only. A pose
        // target here needs IK and RRTConnect could not sample a collision-free
        // goal; a joint goal cannot.
        const bool preorient_ok = preorientWrists(err);
        {
            bool hd = false;
            if (!lastToolGrasped(hd) && hd) {
                transporting_.store(false);
                abortHoldingAndGoHome("tool dropped during pre-orient", err);
                return false;
            }
        }
        if (!preorient_ok) {
            transporting_.store(false);
            err = "pre-orient wrists: " + err;
            return false;
        }

        transporting_.store(false);

        // Remember where this tool came from so return_tool can put it back.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_pick_grasp_pose_ = grasp_pose;
            last_pick_approach_pose_ = approach_pose;
            have_last_pick_ = true;
        }
        return true;
    }

    // ── Skill: HandoverTool ──────────────────────────────────────────

    bool planHandoverToPose(
        const geometry_msgs::msg::Pose &hand_pose,
        moveit::planning_interface::MoveGroupInterface::Plan &plan,
        geometry_msgs::msg::Pose &target,
        std::string &err) {
        target = hand_pose;
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

        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->clearPoseTargets();
            err = "handover plan failed";
            return false;
        }
        move_group_->clearPoseTargets();
        return true;
    }

    bool doHandover(const std::shared_ptr<GoalHandleHandover> &goal_handle,
                    std::string &err) {
        const auto goal = goal_handle->get_goal();
        const geometry_msgs::msg::PoseStamped &goal_pose = goal->hand_pose;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::msg::Pose target;

        if (!goal_pose.header.frame_id.empty()) {
            // Explicit pose bypass (manual testing) — skip the gesture wait.
            RCLCPP_INFO(get_logger(), "Hand detected: using explicit HandoverTool goal pose.");
            if (!planHandoverToPose(goal_pose.pose, plan, target, err)) {
                return false;
            }
            RCLCPP_INFO(get_logger(), "Reachability decision: reachable.");
        } else {
            // Gesture-triggered path: hold the tool and wait until a
            // reachable gesture arrives. Unreachable gestures are ignored
            // before any state, feedback, gripper, or motion side effect.
            RCLCPP_INFO(get_logger(),
                "HandoverTool: tool ready — waiting for double_open_close gesture.");
            // Open the gesture gate: hand_tracker only publishes /hand_gesture
            // while this is true, so stray gestures outside a handover are
            // ignored. Reset to false unconditionally in handoverExecute.
            publishHandoverWaiting(true);
            while (rclcpp::ok()) {
                publishHandoverFeedback(goal_handle, "AWAITING_GESTURE");
                if (!waitForGesture(goal_handle)) {
                    // Tool fell out while waiting? Treat as a loss and retry.
                    bool have_grasp_data = false;
                    if (!lastToolGrasped(have_grasp_data) && have_grasp_data) {
                        abortHoldingAndGoHome("gripper empty during handover", err);
                        return false;
                    }
                    bool preempted = false;
                    {
                        std::lock_guard<std::mutex> lock(gesture_mutex_);
                        preempted = abort_handover_;
                    }
                    err = goal_handle->is_canceling()
                          ? "handover cancelled while waiting for gesture"
                          : preempted
                              ? "handover preempted while waiting for gesture"
                              : "no double_open_close gesture detected";
                    return false;
                }

                RCLCPP_INFO(get_logger(),
                    "HandoverTool: gesture detected — capturing open-hand pose.");
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(std::max(0.0, post_gesture_settle_sec_))));

                if (goal_handle->is_canceling()) {
                    err = "handover cancelled after gesture";
                    return false;
                }
                {
                    std::lock_guard<std::mutex> lock(gesture_mutex_);
                    if (abort_handover_) {
                        err = "handover preempted after gesture";
                        return false;
                    }
                }

                geometry_msgs::msg::Pose hand_pose;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (!have_hand_state_ || !last_hand_state_.is_tracked) {
                        err = "hand not tracked after gesture";
                        return false;
                    }
                    hand_pose = last_hand_state_.hand_pose.pose;
                }
                RCLCPP_INFO(get_logger(), "Hand detected: using tracked hand pose after gesture.");

                std::string plan_err;
                if (!planHandoverToPose(hand_pose, plan, target, plan_err)) {
                    RCLCPP_WARN(get_logger(), "Reachability decision: unreachable.");
                    RCLCPP_WARN(get_logger(),
                        "Action rejected: handover target unreachable; ignoring gesture and waiting for next gesture.");
                    continue;
                }
                RCLCPP_INFO(get_logger(), "Reachability decision: reachable.");
                break;
            }
            if (!rclcpp::ok()) {
                err = "shutdown while waiting for gesture";
                return false;
            }
        }

        std::string tool_id_snapshot, tool_class_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            tool_id_snapshot = active_tool_id_;
            tool_class_snapshot = active_tool_class_;
        }

        // Make sure the tool is still in the gripper before driving to the hand.
        // The loss monitor flips /tool_grasped to false if it fell out meanwhile.
        bool have_grasp_data = false;
        const bool still_holding = lastToolGrasped(have_grasp_data);
        if (have_grasp_data && !still_holding) {
            abortHoldingAndGoHome("gripper empty before handover", err);
            return false;
        }

        publishState("HANDOVER", tool_id_snapshot, tool_class_snapshot);
        publishHandoverFeedback(goal_handle, "MOVING_TO_HAND");
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "handover execute failed";
            return false;
        }

        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, pre_release_dwell_seconds_))));

        publishState("RELEASING", tool_id_snapshot, tool_class_snapshot);
        publishGripperZeroer(true);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_zeroer_settle_seconds_))));

        // Wait indefinitely for the surgeon to take the tool — no timeout.
        if (!waitForGripperDoneIndefinite(goal_handle)) {
            publishGripperZeroer(false);
            err = "handover cancelled while waiting for the surgeon to take the tool";
            return false;
        }
        publishGripperZeroer(false);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_open_pause_seconds_))));

        // The surgeon has physically pulled the tool out of the jaws (/gripper_done
        // fired on the force-guided release). ONLY now is it his — and from here it
        // is invisible to every camera, which is entirely normal.
        publishHeldToolEvent("HANDED_OVER");

        // Tool delivered — drop the attached collision box and forget the pick.
        detachToolBox();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_pick_ = false;
        }

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
        detachToolBox();
        publishHeldToolEvent("RELEASED");
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_pick_ = false;
        }
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

    // ── Place primitive (shared by ReturnTool and ReturnToolHome) ────

    // Put the held tool down at release_pose and let go. Plans all three legs
    // before moving, exactly as ReturnTool always has.
    bool placeToolAt(const geometry_msgs::msg::Pose &approach_pose,
                     const geometry_msgs::msg::Pose &release_pose,
                     std::string &err) {
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan,
            descend_plan, lift_plan;
        move_group_->setStartStateToCurrentState();
        if (!planPoseTarget(approach_pose, approach_plan, err)) {
            err = "place approach plan: " + err; return false;
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(approach_plan));
        if (!planLinearPose(release_pose, descend_plan, err)) {
            err = "place descend plan: " + err; return false;
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(descend_plan));
        if (!planLinearPose(approach_pose, lift_plan, err)) {
            err = "place lift plan: " + err; return false;
        }

        RCLCPP_INFO(get_logger(), "Placing at (%.3f, %.3f, %.3f).",
            release_pose.position.x, release_pose.position.y,
            release_pose.position.z);

        if (!executePlan(approach_plan, err)) { err = "place approach exec: " + err; return false; }
        if (!executePlan(descend_plan, err))  { err = "place descend exec: "  + err; return false; }
        publishGripper(true);  // open — release the tool
        sleepForGripper();
        detachToolBox();
        if (!executePlan(lift_plan, err))     { err = "place lift exec: "     + err; return false; }
        return true;
    }

    // ── Skill: ReturnTool ────────────────────────────────────────────

    // Brings the currently-held tool back to where it was picked from and
    // releases it return_release_height_m_ above the original grasp pose.
    bool doReturnTool(std::string &err) {
        geometry_msgs::msg::Pose grasp_pose, approach_pose;
        std::string tool_id_snapshot, tool_class_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_last_pick_) {
                err = "no recorded pick to return";
                return false;
            }
            grasp_pose = last_pick_grasp_pose_;
            approach_pose = last_pick_approach_pose_;
            tool_id_snapshot = active_tool_id_;
            tool_class_snapshot = active_tool_class_;
        }

        publishState("RETURNING", tool_id_snapshot, tool_class_snapshot);

        geometry_msgs::msg::Pose release_pose = grasp_pose;
        release_pose.position.z += return_release_height_m_;

        if (!placeToolAt(approach_pose, release_pose, err)) return false;

        // Back where it was picked from — for the instrument tray, that is its home.
        publishHeldToolEvent("PLACED_HOME");

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_pick_ = false;
        }

        std::string home_err;
        if (!doReturnHomeInternal(home_err)) {
            err = "return-home after return-tool failed: " + home_err;
            return false;
        }
        return true;
    }

    // ── Skill: ReturnToolHome ────────────────────────────────────────
    // Take a tool the surgeon parked on the reclaim tray and put it back on its
    // home slot on the instrument tray.

    // Ask the registry where a tool of this class belongs.
    bool fetchToolHome(const std::string &tool_class,
                       const std::string &track_id,
                       tracking_msgs::srv::GetToolHome::Response &out,
                       std::string &err) {
        if (!tool_home_client_->wait_for_service(std::chrono::seconds(2))) {
            err = "/get_tool_home service not available";
            return false;
        }
        auto req = std::make_shared<tracking_msgs::srv::GetToolHome::Request>();
        req->tool_class = tool_class;
        req->track_id = track_id;
        auto future = tool_home_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            err = "/get_tool_home timed out";
            return false;
        }
        auto resp = future.get();
        if (!resp->success) {
            err = resp->message;
            return false;
        }
        out = *resp;
        return true;
    }

    // THE geometry of putting a tool back. Read this before changing it.
    //
    // The grasp point is handle_center + d * functional_end_dir, and d is NOT the
    // same on both trays: the sliding-into-the-opening strategy picks a different
    // spot along the shaft depending on that tray's hole. So the gripper is holding
    // this tool at a different point than it did when the tool sat at home.
    //
    // Placing the gripper at the home GRASP pose would therefore leave the tool
    // displaced along its own axis by (d_reclaim - d_home) — centimetres, for a
    // pair of scissors. What has to be reproduced is the TOOL pose, not the grasp:
    //
    //   TCP -> home_handle_center + d_now * home_functional_end_dir
    //
    // which lands the tool's handle exactly back on its home handle position.
    void computePlacePose(const tracking_msgs::srv::GetToolHome::Response &home,
                          double grasp_distance_m,
                          geometry_msgs::msg::Pose &release_out,
                          geometry_msgs::msg::Pose &approach_out) {
        const double nx = home.home_functional_end_dir.x;
        const double ny = home.home_functional_end_dir.y;
        const double n = std::hypot(nx, ny);
        const double ux = (n > 1e-9) ? nx / n : 1.0;
        const double uy = (n > 1e-9) ? ny / n : 0.0;

        release_out.position.x = home.home_handle_center.x + grasp_distance_m * ux;
        release_out.position.y = home.home_handle_center.y + grasp_distance_m * uy;
        release_out.position.z = home.home_plane_z + return_release_height_m_;

        // The tool must also end up along its home axis. handle_axis points from the
        // functional end back toward the handle, i.e. the negated end direction.
        geometry_msgs::msg::Vector3 handle_axis;
        handle_axis.x = -ux;
        handle_axis.y = -uy;
        handle_axis.z = 0.0;
        release_out.orientation =
            topDownQuaternionFromHandleAxis(handle_axis, tool_yaw_offset_rad_);

        approach_out = release_out;
        approach_out.position.z = release_out.position.z + approach_height_m_;
    }

    bool doReturnToolHome(const std::string &tool_id_arg,
                          std::vector<std::string> &returned_out,
                          std::vector<std::string> &skipped_ids_out,
                          std::vector<std::string> &skipped_reasons_out,
                          std::string &err) {
        // Which reclaim-tray tools are we dealing with?
        std::vector<tracking_msgs::msg::GraspCandidate> candidates;
        if (!collectCandidates(candidates, kReclaimLocation, err)) return false;
        if (!tool_id_arg.empty()) {
            candidates.erase(
                std::remove_if(candidates.begin(), candidates.end(),
                    [&](const auto &c) { return c.tool_id != tool_id_arg; }),
                candidates.end());
            if (candidates.empty()) {
                err = "tool_id '" + tool_id_arg + "' not on the reclaim tray";
                return false;
            }
        }

        for (const auto &cand : candidates) {
            // Ask where it belongs BEFORE touching it. A tool with no known home is
            // left exactly where it is — better a tool still on the reclaim tray
            // than one put down at a guessed spot.
            tracking_msgs::srv::GetToolHome::Response home;
            std::string home_err;
            if (!fetchToolHome(cand.tool_class, cand.tool_id, home, home_err)) {
                RCLCPP_WARN(get_logger(), "No home for %s (%s): %s — leaving it.",
                            cand.tool_id.c_str(), cand.tool_class.c_str(),
                            home_err.c_str());
                skipped_ids_out.push_back(cand.tool_id);
                skipped_reasons_out.push_back(home_err);
                continue;
            }

            tracking_msgs::msg::GraspCandidate chosen;
            geometry_msgs::msg::Pose grasp_pose, approach_pose;
            std::string grasp_err;
            if (!graspToolCore(cand.tool_id, kReclaimLocation, reclaim_hold_z_m_,
                               chosen, grasp_pose, approach_pose, grasp_err)) {
                RCLCPP_WARN(get_logger(), "Could not grasp %s: %s",
                            cand.tool_id.c_str(), grasp_err.c_str());
                skipped_ids_out.push_back(cand.tool_id);
                skipped_reasons_out.push_back(grasp_err);
                std::string home_ret;
                doReturnHomeInternal(home_ret);
                continue;
            }

            // Where along the tool are we holding it? THIS is what the place pose
            // has to be built from — not the pose we grasped at.
            const double d = std::hypot(
                grasp_pose.position.x - chosen.handle_center.x,
                grasp_pose.position.y - chosen.handle_center.y);

            geometry_msgs::msg::Pose release_pose, place_approach;
            computePlacePose(home, d, release_pose, place_approach);

            publishState("RETURNING", chosen.tool_id, chosen.tool_class);
            RCLCPP_INFO(get_logger(),
                "ReturnToolHome: %s (%s) -> slot %s. Holding it %.1f mm along its "
                "shaft, so releasing at (%.3f, %.3f, %.3f).",
                chosen.tool_id.c_str(), chosen.tool_class.c_str(),
                home.slot_id.c_str(), d * 1000.0,
                release_pose.position.x, release_pose.position.y,
                release_pose.position.z);

            std::string place_err;
            if (!placeToolAt(place_approach, release_pose, place_err)) {
                // Still holding it. Do NOT drop it here — put it back on the reclaim
                // tray, where it at least stays findable.
                std::string back_err;
                if (!placeToolAt(approach_pose, grasp_pose, back_err)) {
                    err = "could not place " + chosen.tool_id + " at its home ("
                          + place_err + ") and could not put it back either ("
                          + back_err + ") — the tool is still in the gripper";
                    return false;
                }
                skipped_ids_out.push_back(cand.tool_id);
                skipped_reasons_out.push_back("home unreachable: " + place_err);
                std::string home_ret;
                doReturnHomeInternal(home_ret);
                continue;
            }

            publishHeldToolEvent("PLACED_HOME");
            returned_out.push_back(home.slot_id);

            std::string home_ret;
            doReturnHomeInternal(home_ret);
        }

        if (returned_out.empty() && !skipped_ids_out.empty()) {
            err = "nothing returned; " + std::to_string(skipped_ids_out.size())
                  + " skipped";
            return false;
        }
        return true;
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

    // ── Action: GraspTool ────────────────────────────────────────────
    // Bare grasp: pick the tool up and hold it there. No present rotation, no
    // handover. Used to bring tools off the reclaim tray, and as the first leg
    // of putting a used tool back on the instrument tray.

    rclcpp_action::GoalResponse graspHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GraspTool::Goal> goal) {
        RCLCPP_INFO(get_logger(),
                    "GraspTool goal received: tool_id='%s' location='%s'",
                    goal->tool_id.c_str(),
                    goal->location.empty() ? kReclaimLocation
                                           : goal->location.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse graspHandleCancel(
        const std::shared_ptr<GoalHandleGrasp>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void graspHandleAccepted(const std::shared_ptr<GoalHandleGrasp> goal_handle) {
        std::thread([this, goal_handle] { this->graspExecute(goal_handle); }).detach();
    }
    void graspExecute(const std::shared_ptr<GoalHandleGrasp> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<GraspTool::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        const auto goal = goal_handle->get_goal();
        const std::string location =
            goal->location.empty() ? kReclaimLocation : goal->location;

        // The reclaim tray sits down inside a bracket; rise clear of it before
        // the tool box is attached. The instrument tray does not need it.
        const double clearout = (location == kReclaimLocation)
            ? reclaim_hold_z_m_
            : std::numeric_limits<double>::quiet_NaN();

        tracking_msgs::msg::GraspCandidate chosen;
        geometry_msgs::msg::Pose grasp_pose, approach_pose;
        std::string err;
        const bool ok = graspToolCore(goal->tool_id, location, clearout, chosen,
                                      grasp_pose, approach_pose, err);
        result->success = ok;
        result->grasped_tool_id = chosen.tool_id;
        result->grasped_tool_class = chosen.tool_class;
        if (ok) {
            result->message = "ok";
            // The arm stops here, holding the tool. HOLDING (not TRANSPORTING)
            // so toolGraspedCb does not treat a gripper report as an in-transit
            // drop and stop a motion that is not running.
            publishState("HOLDING", chosen.tool_id, chosen.tool_class);
            RCLCPP_INFO(get_logger(),
                "GraspTool: holding %s (%s) above %s. Arm idle.",
                chosen.tool_id.c_str(), chosen.tool_class.c_str(),
                location.c_str());
            goal_handle->succeed(result);
        } else {
            result->message = err;
            publishState("IDLE", "", "");
            goal_handle->abort(result);
        }
    }

    // ── Action: ReturnToolHome ───────────────────────────────────────

    rclcpp_action::GoalResponse returnHomeSlotHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReturnToolHome::Goal> goal) {
        RCLCPP_INFO(get_logger(), "ReturnToolHome goal: tool_id='%s'%s",
                    goal->tool_id.c_str(),
                    goal->tool_id.empty() ? " (all reclaim tools)" : "");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse returnHomeSlotHandleCancel(
        const std::shared_ptr<GoalHandleReturnHome_>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void returnHomeSlotHandleAccepted(
        const std::shared_ptr<GoalHandleReturnHome_> goal_handle) {
        std::thread([this, goal_handle] {
            this->returnHomeSlotExecute(goal_handle);
        }).detach();
    }
    void returnHomeSlotExecute(
        const std::shared_ptr<GoalHandleReturnHome_> goal_handle) {
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        auto result = std::make_shared<ReturnToolHome::Result>();
        if (!lock.owns_lock()) {
            result->success = false;
            result->message = "another skill is currently executing";
            goal_handle->abort(result);
            return;
        }
        const auto goal = goal_handle->get_goal();
        std::vector<std::string> returned, skipped, reasons;
        std::string err;
        const bool ok = doReturnToolHome(
            goal->tool_id, returned, skipped, reasons, err);

        result->success = ok;
        result->returned_slot_ids = returned;
        result->skipped_tool_ids = skipped;
        result->skipped_reasons = reasons;
        result->message = ok ? ("returned " + std::to_string(returned.size())) : err;
        publishState("IDLE", "", "");
        if (ok) {
            goal_handle->succeed(result);
        } else {
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
        std::string err;
        const bool ok = doHandover(goal_handle, err);
        result->success = ok;
        result->message = ok ? "ok" : err;
        // Close the gesture gate on every exit (success, abort, cancel,
        // timeout, preemption).
        publishHandoverWaiting(false);
        publishState("IDLE", "", "");
        if (ok) {
            goal_handle->succeed(result);
        } else if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->abort(result);
        }
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

    // ── Action: ReturnTool ───────────────────────────────────────────

    rclcpp_action::GoalResponse returnToolHandleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ReturnTool::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse returnToolHandleCancel(
        const std::shared_ptr<GoalHandleReturnTool>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void returnToolHandleAccepted(
        const std::shared_ptr<GoalHandleReturnTool> goal_handle) {
        std::thread([this, goal_handle] { this->returnExecute(goal_handle); }).detach();
    }
    void returnExecute(const std::shared_ptr<GoalHandleReturnTool> goal_handle) {
        // Preempt a running handover: tell waitForGesture to stop, then take
        // over the executor as soon as that handover releases the mutex.
        {
            std::lock_guard<std::mutex> lock(gesture_mutex_);
            abort_handover_ = true;
        }
        gesture_cv_.notify_all();

        std::unique_lock<std::mutex> lock(execution_mutex_);  // blocking
        {
            std::lock_guard<std::mutex> glock(gesture_mutex_);
            abort_handover_ = false;
        }

        auto result = std::make_shared<ReturnTool::Result>();
        std::string err;
        const bool ok = doReturnTool(err);
        result->success = ok;
        result->message = ok ? "ok" : err;
        publishState("IDLE", "", "");
        if (ok) {
            goal_handle->succeed(result);
        } else if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->abort(result);
        }
    }

    // ── Members ──────────────────────────────────────────────────────

    // Params
    double z_offset_m_;
    double reclaim_z_offset_m_;
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
    double grasp_check_timeout_sec_;
    double post_lift_settle_sec_;
    int max_regrasp_retries_;
    double regrasp_deeper_step_m_;
    double regrasp_center_step_m_;
    double cartesian_min_fraction_;
    double gesture_wait_timeout_sec_;
    double post_gesture_settle_sec_;
    double return_release_height_m_;
    double reclaim_hold_z_m_;
    double present_shoulder_pan_rad_;
    double present_wrist1_rad_;
    double present_wrist2_rad_;
    double present_wrist3_rad_;
    std::vector<std::string> joint_state_names_;
    geometry_msgs::msg::Point hand_offset_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    std::vector<double> home_joints_;

    // Pubs / subs
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr verify_grasp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr handover_waiting_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<tracking_msgs::msg::ToolEvent>::SharedPtr tool_event_pub_;
    rclcpp::Client<tracking_msgs::srv::GetToolHome>::SharedPtr tool_home_client_;
    rclcpp_action::Server<ReturnToolHome>::SharedPtr return_home_slot_srv_;
    rclcpp::Subscription<tracking_msgs::msg::HandState>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tool_grasped_sub_;
    rclcpp::Client<tracking_msgs::srv::GetWorldState>::SharedPtr world_state_client_;

    // Action servers
    rclcpp_action::Server<PickTool>::SharedPtr pick_srv_;
    rclcpp_action::Server<GraspTool>::SharedPtr grasp_srv_;
    rclcpp_action::Server<HandoverTool>::SharedPtr handover_srv_;
    rclcpp_action::Server<ReleaseTool>::SharedPtr release_srv_;
    rclcpp_action::Server<ReturnHome>::SharedPtr home_srv_;
    rclcpp_action::Server<ReturnTool>::SharedPtr return_srv_;

    // MoveIt
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface psi_;

    // State
    std::mutex execution_mutex_;
    std::mutex state_mutex_;
    std::mutex gripper_done_mutex_;
    std::condition_variable gripper_done_cv_;
    bool gripper_done_received_ = false;
    std::mutex tool_grasped_mutex_;
    std::condition_variable tool_grasped_cv_;
    bool tool_grasped_received_ = false;
    bool tool_grasped_value_ = false;
    bool tool_grasped_ever_ = false;
    // True while the arm is transporting a grasped tool (present/pre-orient).
    // A /tool_grasped=false during this window stops the running motion.
    std::atomic<bool> transporting_{false};
    std::mutex gesture_mutex_;
    std::condition_variable gesture_cv_;
    bool gesture_received_ = false;
    bool abort_handover_ = false;
    tracking_msgs::msg::HandState last_hand_state_;
    bool have_hand_state_ = false;
    std::string current_state_ = "IDLE";
    std::string active_tool_id_;
    std::string active_tool_class_;
    // Last successful pick — used by return_tool to put a wrong tool back.
    geometry_msgs::msg::Pose last_pick_grasp_pose_;
    geometry_msgs::msg::Pose last_pick_approach_pose_;
    bool have_last_pick_ = false;

    // Identity + tool pose of whatever is in the gripper, so a later event (the
    // handover completing, a drop) can still say WHICH tool it was.
    std::string held_track_id_;
    std::string held_tool_class_;
    std::string held_from_location_;
    geometry_msgs::msg::Point held_handle_center_;
    geometry_msgs::msg::Vector3 held_end_dir_;
    double held_grasp_distance_m_ = 0.0;
    bool have_held_tool_ = false;
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
