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
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
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

#include "tracking_pkg/elbow_up_guard.hpp"
#include "tracking_pkg/preflight_retry.hpp"
#include "tracking_pkg/return_home_route.hpp"

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
#include <optional>
#include <stdexcept>
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
using HomeReturnOrigin = tracking_pkg::execution::HomeReturnOrigin;

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
        // Height above an instrument-tray slot the arm reaches — and finishes
        // rotating to the placement orientation — BEFORE the final descent, so a
        // tool never rotates low among its neighbours and sweeps them. The
        // descent from here is a pure vertical cartesian move. Bigger = more
        // clearance; tune if a long tool still grazes while rotating.
        place_rotate_height_m_ = declare_parameter("place_rotate_height_m", 0.15);
        tool_yaw_offset_rad_ = declare_parameter("tool_yaw_offset_rad", 1.57079632679);
        // Dynamic instrument-tray IK must stay on the elbow-up branch. A pose
        // target otherwise lets KDL/RRTConnect choose the negative branch, where
        // the held-tool box can already intersect upper_arm_link after the lift.
        instrument_pick_elbow_min_rad_ = declare_parameter(
            "instrument_pick.elbow_min_rad", 0.0);
        if (!tracking_pkg::execution::isValidInstrumentElbowMinimum(
                instrument_pick_elbow_min_rad_)) {
            const std::string message =
                "instrument_pick.elbow_min_rad must be finite and satisfy "
                "0.0 <= value < pi";
            RCLCPP_FATAL(get_logger(), "%s", message.c_str());
            throw std::invalid_argument(message);
        }
        // The held tool's collision box, measured FROM THE JAWS along the tool —
        // not from its middle. See attachToolBox().
        tool_box_handle_m_ = declare_parameter("tool_box_handle_m", 0.08);
        tool_box_tip_m_ = declare_parameter("tool_box_tip_m", 0.18);
        tool_box_width_m_ = declare_parameter("tool_box_width_m", 0.05);
        tool_box_height_m_ = declare_parameter("tool_box_height_m", 0.05);
        // OMPL planning is stochastic and its post-smoothing occasionally
        // produces a path that fails validation ("path found but invalid") —
        // one observed contact away from a perfectly plannable motion. Re-plan
        // this many times in total before a failure is treated as real.
        plan_attempts_ = static_cast<int>(
            declare_parameter("plan_attempts", 3));
        // Reclaim picks start from a taught lower staging pose. If one complete
        // approach->descend->lift pre-flight fails, plan the whole sequence once
        // more from that unchanged live pose before paying for a retreat home.
        reclaim_preflight_attempts_ = static_cast<int>(
            declare_parameter("reclaim_preflight_attempts", 2));
        if (reclaim_preflight_attempts_ < 1) {
            const std::string message =
                "reclaim_preflight_attempts must be at least 1";
            RCLCPP_FATAL(get_logger(), "%s", message.c_str());
            throw std::invalid_argument(message);
        }
        // Extra height added ONCE to the reclaim-exit rise (exitReclaimToUpper),
        // so the held_tool box clears the reclaim tray when the base later swings
        // across — in a single lift, not a separate second hop. Raise if the
        // transit ever contacts the reclaim tray again.
        reclaim_exit_clearance_m_ = declare_parameter(
            "reclaim_exit_clearance_m", 0.05);
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
        // The straight-up lift after a grasp only has to raise the tool clear of
        // the tray, not reach the exact approach pose. A thin tool near a rail
        // can graze the gripper in the last few mm (e.g. 89 % of a 4 cm lift);
        // accepting a shorter-but-still-clearing straight lift lets the pick
        // proceed instead of rejecting an obviously graspable tool. It stops
        // safely before the graze; the present/transit is planned fresh after.
        // Approach and descend stay strict (they must reach the tool exactly).
        lift_min_fraction_ = declare_parameter("lift_min_fraction", 0.6);
        // Handover waits for the surgeon's double_open_close gesture before
        // moving to the hand. 0.0 = wait indefinitely.
        gesture_wait_timeout_sec_ = declare_parameter("gesture_wait_timeout_sec", 0.0);
        post_gesture_settle_sec_ = declare_parameter("post_gesture_settle_sec", 0.5);
        // return_tool releases the wrong tool this far above the pickup pose.
        return_release_height_m_ = declare_parameter("return_release_height_m", 0.005);
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

        // home is the hub: over the instrument tray, right of centre. Every
        // instrument tool is reachable from here by direct planning, a short pan
        // rotation reaches the handover pose, and it is the taught pivot the arm
        // routes through to place on the right side of the tray. The arm drives
        // here once at launch (see main()).
        home_joints_ = parameterVectorOrDefault(
            *this, "home_joints",
            {0.7702576518, -1.9044758282, 1.8983271758,
             -1.5910726986, -1.5716832320, 0.8188708425},
            6);
        // Taught transit pose over the LEFT side of the instrument tray, the
        // counterpart of home (which sits over the right side). Placing a tool back
        // on a left slot routes through here instead of home, so the arm crosses to
        // the left directly rather than swinging out to the right and back.
        instrument_left_stage_joints_ = parameterVectorOrDefault(
            *this, "instrument_left_stage_joints",
            {2.1318871975, -1.1601789457, 1.1124246756,
             -1.5276912202, -1.5967219512, 2.1804935932},
            6);
        // Hardware-taught, tool-clear pose reached directly after the 4 cm
        // reclaim lift by ReturnToolHome. Unlike the generic reclaim upper pad,
        // this is a deterministic six-joint goal and is part of the pre-flight
        // trajectory chain planned before the gripper closes.
        instrument_stage_joints_ = declare_parameter(
            "instrument_stage_joints",
            std::vector<double>{
                4.8766698837, -1.1527752441, 1.1332219283,
                -1.5510326673, -1.5708482901, -2.9439778964});
        if (!tracking_pkg::execution::isValidSixJointPose(
                instrument_stage_joints_)) {
            const std::string message =
                "instrument_stage_joints must contain exactly six finite values";
            RCLCPP_FATAL(get_logger(), "%s", message.c_str());
            throw std::invalid_argument(message);
        }

        // ── Reclaim staging ──────────────────────────────────────────
        // The reclaim tray sits under a 60 cm camera post, so the arm cannot fly
        // straight at a tool there or lift straight up out of it. Instead it enters
        // and leaves through two taught joint poses over the tray: an upper one
        // (clear of the post, where the held-tool box is attached) and a lower one
        // (just above the tools, the launch pad the grasp descends from). Both are
        // joint angles — no IK, no sampling. Teach by jogging + read_stage_pose.py.
        reclaim_stage_upper_joints_ = parameterVectorOrDefault(
            *this, "reclaim_stage_upper_joints",
            {4.8814082146, -1.1096825761, 1.2962282340,
             -1.7339645825, -1.5696294943, 0.1715736389},
            6);
        reclaim_stage_lower_joints_ = parameterVectorOrDefault(
            *this, "reclaim_stage_lower_joints",
            {4.8818922043, -0.8805474800, 1.6040924231,
             -2.2710281811, -1.5696328322, 0.1718008518},
            6);
        // The world-x boundary between the left and right side of the instrument
        // tray (tray centre = 0.0). A tool picked to the RIGHT of it is presented
        // via home first (from there the present rotation is clear); a tool picked
        // to the left turns toward the surgeon straight away. Only doPick uses this.
        instrument_right_side_x_ =
            declare_parameter("instrument_right_side_x", 0.0);
        // Tool classes whose held-tool collision box is built REVERSED: the long
        // reach toward the handle, the short toward the tip. For tools gripped near
        // their functional end (the hammer, grasped close to its head) the body
        // runs backward from the jaws, not forward — see attachToolBox.
        reversed_tool_box_classes_ = declare_parameter(
            "reversed_tool_box_classes", std::vector<std::string>{"hammer"});

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
        // Handover events for the HRI display + (dormant) audio feedback.
        handover_event_pub_ = create_publisher<std_msgs::msg::String>(
            "/handover_event", 10);

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

    // Drive the arm to home once at startup, so it always begins from the hub
    // pose the whole motion model assumes. Call from main() after the executor is
    // spinning (getCurrentState / plan need it). Best-effort: a failure is logged
    // loudly but does not abort the node.
    void goHomeOnStartup() {
        // Wait for the controllers and the state monitor to come up.
        for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
            if (move_group_ && move_group_->getCurrentState(0.5)) break;
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        std::lock_guard<std::mutex> lock(execution_mutex_);
        RCLCPP_INFO(get_logger(), "Startup: moving to home.");
        std::string err;
        if (!moveToJointPositions(home_joints_, err)) {
            RCLCPP_WARN(get_logger(),
                "Startup home move failed (%s). The arm is NOT at home — move it "
                "there before issuing a pick.", err.c_str());
            return;
        }
        publishIdleUnlessRecoveryError();
        RCLCPP_INFO(get_logger(), "Startup: at home, ready.");
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

        publishIdleUnlessRecoveryError();
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
                || state == "PRESENTING" || state == "HOLDING"
                || (state == "RECOVERY_ERROR" && !tool_id.empty())) {
                active_tool_id_ = tool_id;
                active_tool_class_ = tool_class;
            } else if (state == "IDLE" || state == "RECOVERY_ERROR") {
                active_tool_id_ = "";
                active_tool_class_ = "";
            }
        }
    }

    void publishIdleUnlessRecoveryError() {
        if (!recovery_error_.load()) {
            publishState("IDLE", "", "");
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

    // plan() with up to plan_attempts_ tries. OMPL sampling is stochastic and
    // its post-smoothing sometimes yields a "path found but invalid" abort that
    // a fresh attempt clears; without this, one such flake mid-sequence (e.g.
    // the reclaim→instrument transit of return_tool_home) aborted the whole
    // skill. `resync_start` re-reads the live robot state between attempts —
    // callers that plan from a constructed start state must pass false.
    bool planWithRetry(moveit::planning_interface::MoveGroupInterface::Plan &plan_out,
                       bool resync_start, std::string &err) {
        for (int attempt = 1; attempt <= plan_attempts_; ++attempt) {
            if (move_group_->plan(plan_out) ==
                    moveit::core::MoveItErrorCode::SUCCESS) {
                return true;
            }
            if (attempt < plan_attempts_) {
                RCLCPP_WARN(get_logger(),
                    "Plan attempt %d/%d failed — re-planning (OMPL is "
                    "stochastic).", attempt, plan_attempts_);
                if (resync_start) move_group_->setStartStateToCurrentState();
            }
        }
        err = "planning failed after " + std::to_string(plan_attempts_)
              + " attempts";
        return false;
    }

    bool moveToPoseTarget(const geometry_msgs::msg::Pose &pose,
                          std::string &err) {
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!planWithRetry(plan, true, err)) {
            err = "pose target: " + err;
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
                        std::string &err,
                        const moveit_msgs::msg::Constraints *path_constraints = nullptr) {
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        // Never inherit a constraint from an earlier plan. The instrument
        // elbow-up constraint is scoped to this one planning request.
        move_group_->clearPathConstraints();
        if (path_constraints) {
            move_group_->setPathConstraints(*path_constraints);
        }
        move_group_->setPoseTarget(pose);
        // No start-state resync on retry: the caller set it (pre-flight chains
        // plan from the end of the previous leg, not from the live robot).
        const bool ok = planWithRetry(plan_out, false, err);
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();
        if (!ok) err = "planning to pose target failed: " + err;
        return ok;
    }

    bool planLinearPose(const geometry_msgs::msg::Pose &pose,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan_out,
                        std::string &err,
                        double required_fraction = -1.0,
                        const moveit_msgs::msg::Constraints *path_constraints = nullptr) {
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        std::vector<geometry_msgs::msg::Pose> waypoints{pose};
        moveit_msgs::msg::RobotTrajectory traj;
        const double fraction = path_constraints
            ? move_group_->computeCartesianPath(
                waypoints, 0.005, 0.0, traj, *path_constraints, true)
            : move_group_->computeCartesianPath(
                waypoints, 0.005, 0.0, traj);
        const double minimum = required_fraction >= 0.0
                                   ? required_fraction
                                   : cartesian_min_fraction_;
        if (fraction < minimum) {
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

    // Joint-space counterpart to planPoseTarget(). The caller supplies either
    // the live start state or a constructed future state. ReturnToolHome uses
    // the latter so the entire post-lift exit can be validated before closing.
    bool planJointPositions(
        const std::vector<double> &joints,
        moveit::planning_interface::MoveGroupInterface::Plan &plan_out,
        std::string &err,
        bool resync_start) {
        if (joint_state_names_.size() != joints.size()) {
            err = "joint name/value size mismatch";
            return false;
        }
        std::map<std::string, double> target;
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            target[joint_state_names_[i]] = joints[i];
        }
        move_group_->clearPoseTargets();
        move_group_->setPlanningTime(2.0);
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setJointValueTarget(target);
        if (!planWithRetry(plan_out, resync_start, err)) {
            err = "joint plan failed: " + err;
            return false;
        }
        return true;
    }

    bool jointValuesFromPlanEnd(
        const moveit::planning_interface::MoveGroupInterface::Plan &plan,
        std::vector<double> &joints_out,
        std::string &err) const {
        const auto &trajectory = plan.RSN_PLAN_TRAJECTORY.joint_trajectory;
        if (trajectory.points.empty()) {
            err = "planned trajectory has no points";
            return false;
        }
        const auto &positions = trajectory.points.back().positions;
        if (positions.size() != trajectory.joint_names.size()) {
            err = "planned trajectory joint name/value size mismatch";
            return false;
        }
        joints_out.assign(joint_state_names_.size(), 0.0);
        for (size_t i = 0; i < joint_state_names_.size(); ++i) {
            const auto it = std::find(
                trajectory.joint_names.begin(), trajectory.joint_names.end(),
                joint_state_names_[i]);
            if (it == trajectory.joint_names.end()) {
                err = "planned trajectory is missing joint '" +
                      joint_state_names_[i] + "'";
                return false;
            }
            joints_out[i] = positions[static_cast<size_t>(
                std::distance(trajectory.joint_names.begin(), it))];
        }
        return true;
    }

    bool moveToJointPositions(const std::vector<double> &joints,
                              std::string &err) {
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!planJointPositions(joints, plan, err, true)) {
            return false;
        }
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "joint execute failed";
            return false;
        }
        return true;
    }

    // Cross to another side of the workspace tamely: first swing ONLY the base
    // (shoulder_pan) to the target's pan, holding the rest of the arm's shape, then
    // settle the remaining joints. This turns a big side change into a controlled
    // base rotation plus a small reshape, instead of a freely-sampled pose-target
    // RRT that flips the arm around. target[0] is shoulder_pan (canonical order).
    bool moveToJointsViaBaseRotation(const std::vector<double> &target,
                                     std::string &err) {
        if (target.size() != joint_state_names_.size()) {
            err = "target joints must have " +
                  std::to_string(joint_state_names_.size()) + " values";
            return false;
        }
        if (!rotateShoulderPanTo(target[0], err)) return false;
        return moveToJointPositions(target, err);
    }

    // Cross to a transit pose WHILE HOLDING A TOOL, without snapping the wrist to
    // the pose's taught angles — that snap rotates the held tool to a fixed
    // orientation only for the approach to rotate it right back. Base-swing to the
    // transit side (the tool rotates smoothly with the base), then move only to the
    // transit POSITION keeping the current TCP orientation. The approach afterwards
    // is the one and only wrist rotation, straight to the placement orientation.
    // Mirrors exitReclaimToUpper, with a base rotation in front because this crosses
    // sides.
    bool moveAcrossKeepingOrientation(const std::vector<double> &transit,
                                      std::string &err) {
        if (transit.size() != joint_state_names_.size()) {
            err = "transit joints must have " +
                  std::to_string(joint_state_names_.size()) + " values";
            return false;
        }
        if (!rotateShoulderPanTo(transit[0], err)) return false;   // base to the side

        geometry_msgs::msg::Pose target;
        if (!jointsToTcpPose(transit, target, err)) {
            err = "transit FK: " + err; return false;
        }
        target.orientation = move_group_->getCurrentPose(end_effector_link_)
                                 .pose.orientation;                // hold, do not snap

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->setStartStateToCurrentState();
        std::string lin_err;
        if (planLinearPose(target, plan, lin_err) && executePlan(plan, lin_err)) {
            return true;
        }
        RCLCPP_WARN(get_logger(),
            "Transit: straight move not clean (%s), using a planned move that still "
            "holds the grasp orientation.", lin_err.c_str());
        return moveToPoseTarget(target, err);
    }

    // TCP pose (position + orientation of end_effector_link) at a set of joint
    // values, in reference_frame_. Forward kinematics on a scratch RobotState.
    bool jointsToTcpPose(const std::vector<double> &joints,
                         geometry_msgs::msg::Pose &out, std::string &err) {
        if (joints.size() != joint_state_names_.size()) {
            err = "joints must have " +
                  std::to_string(joint_state_names_.size()) + " values";
            return false;
        }
        moveit::core::RobotStatePtr current = move_group_->getCurrentState(2.0);
        if (!current) { err = "could not read current robot state for FK"; return false; }
        moveit::core::RobotState rs(*current);
        for (size_t i = 0; i < joints.size(); ++i) {
            rs.setVariablePosition(joint_state_names_[i], joints[i]);
        }
        rs.update();
        if (!rs.knowsFrameTransform(reference_frame_)) {
            err = "the robot model does not know frame '" + reference_frame_ + "'";
            return false;
        }
        const Eigen::Isometry3d tf =
            rs.getFrameTransform(reference_frame_).inverse() *
            rs.getGlobalLinkTransform(end_effector_link_);
        const Eigen::Quaterniond q(tf.rotation());
        out.position.x = tf.translation().x();
        out.position.y = tf.translation().y();
        out.position.z = tf.translation().z();
        out.orientation.x = q.x();
        out.orientation.y = q.y();
        out.orientation.z = q.z();
        out.orientation.w = q.w();
        return true;
    }

    // ── Reclaim staging ──────────────────────────────────────────────
    // The reclaim tray sits under a 60 cm camera post, so the arm can neither fly
    // straight at a tool there nor lift straight out. It enters and leaves through
    // two taught joint poses: the upper pad (post-clear, where the tool box is
    // (de)attached) and the lower pad (just above the tools, the launch height a
    // grasp/place plans from).

    bool enterReclaimStaging(std::string &err) {
        // Cross from the instrument side to the reclaim side as a base rotation,
        // not a freely-sampled swing that contorts the arm.
        if (!moveToJointsViaBaseRotation(reclaim_stage_upper_joints_, err)) {
            err = "reclaim upper: " + err; return false;
        }
        if (!moveToJointPositions(reclaim_stage_lower_joints_, err)) {
            err = "reclaim lower: " + err; return false;
        }
        return true;
    }

    // Rise from the tool to the upper pad WITHOUT snapping the wrist to the pad's
    // taught orientation — that snap swings the held tool into the tray or its
    // neighbours. Keep the current (grasp) TCP orientation and move only to the
    // upper pad's POSITION, which is clear of the camera post (x ~= 0.196). A
    // cartesian move holds the orientation fixed along the way; if the straight
    // path is not clean, fall back to a pose target whose GOAL orientation is still
    // the current one, so the wrist still does not rotate.
    bool exitReclaimToUpper(std::string &err) {
        geometry_msgs::msg::Pose upper;
        if (!jointsToTcpPose(reclaim_stage_upper_joints_, upper, err)) {
            err = "upper pad FK: " + err; return false;
        }
        // Rise a little above the pad in the SAME move, so a held tool clears the
        // reclaim tray for the later cross-swing without a separate second lift.
        upper.position.z += reclaim_exit_clearance_m_;
        upper.orientation = move_group_->getCurrentPose(end_effector_link_)
                                .pose.orientation;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->setStartStateToCurrentState();
        std::string lin_err;
        if (planLinearPose(upper, plan, lin_err) && executePlan(plan, lin_err)) {
            return true;
        }
        RCLCPP_WARN(get_logger(),
            "Reclaim exit: straight lift not clean (%s), using a planned move that "
            "still holds the grasp orientation.", lin_err.c_str());
        return moveToPoseTarget(upper, err);
    }

    // Best-effort retreat to home after a failed grasp/place. From the reclaim
    // tray that means rising to the upper pad first, then following the same
    // instrument-stage -> Left-Stage -> Home corridor used by ReturnToolHome.
    // From the instrument tray, go straight Home.
    bool retreatToHome(const std::string &location) {
        std::string stage_err;
        if (location == kReclaimLocation) {
            if (!moveToJointPositions(reclaim_stage_upper_joints_, stage_err)) {
                RCLCPP_WARN(get_logger(),
                    "Safe retreat could not reach reclaim upper (%s); "
                    "attempting Home directly from the live state.",
                    stage_err.c_str());
            }
        }
        std::string home_err;
        const HomeReturnOrigin origin = location == kReclaimLocation
            ? HomeReturnOrigin::RECLAIM
            : HomeReturnOrigin::DIRECT;
        if (!doReturnHomeInternal(home_err, origin)) {
            recovery_error_.store(true);
            publishState("RECOVERY_ERROR", "", "");
            RCLCPP_ERROR(get_logger(),
                "Safe retreat failed to reach Home: %s. Executor remains in "
                "RECOVERY_ERROR until return_home succeeds.",
                home_err.c_str());
            return false;
        }
        recovery_error_.store(false);
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
    // The held tool, as the planner sees it: a box along the TCP Y axis (which is
    // the tool's own axis, because the jaws close across it). Attaching it keeps
    // the onward motion — present rotation, handover, place-back — clear of the
    // tray-camera stand, and explains the tool's reach beyond the gripper's own
    // collision geometry.
    //
    // The box is LOPSIDED, and that is the point. The jaws never hold a tool in
    // its middle: the grasp point is handle_center + d * functional_end_dir, so the
    // handle ends a few cm behind the jaws while the working end runs far ahead of
    // them. A box centred on the TCP therefore has to be as long as the longest
    // possible overhang in BOTH directions, and the half that sticks out behind the
    // gripper is pure fiction — it models nothing, and it fouls things the real tool
    // would never touch. Reaching further toward the tip than toward the handle
    // models the same tool with far less phantom volume.

    void attachToolBox(const tracking_msgs::msg::GraspCandidate &cand,
                       const geometry_msgs::msg::Pose &grasp_pose) {
        // Which way along TCP Y does the TIP lie? A lopsided box makes this a real
        // question — get it backwards and the tool is modelled sticking out of the
        // wrong side of the jaws. So derive it instead of assuming it: today the
        // grasp orientation is built from handle_axis (= -functional_end_dir) with
        // tool_yaw_offset_rad, which puts +Y on the handle side — but both of those
        // are conventions that could be re-tuned, and this stays right if they are.
        const auto &q = grasp_pose.orientation;
        const double tcp_y_x = 2.0 * (q.x * q.y - q.w * q.z);
        const double tcp_y_y = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
        const double toward_tip = tcp_y_x * cand.functional_end_dir.x
                                + tcp_y_y * cand.functional_end_dir.y;

        double handle_len = tool_box_handle_m_;
        double tip_len = tool_box_tip_m_;
        // Some tools are gripped near their functional end, so the body runs
        // backward from the jaws (toward the handle), not forward: the long reach
        // has to point at the handle. The hammer, grasped close to its head, is the
        // case. Swapping the two lengths flips the box without touching the
        // tip-vs-handle direction derived above.
        if (std::find(reversed_tool_box_classes_.begin(),
                      reversed_tool_box_classes_.end(), cand.tool_class)
                != reversed_tool_box_classes_.end()) {
            std::swap(handle_len, tip_len);
        }
        double tip_sign = (toward_tip >= 0.0) ? 1.0 : -1.0;
        if (std::abs(toward_tip) < 0.5) {
            // TCP Y does not run along the tool at all, so "toward the tip" has no
            // meaning here and the lopsided box would be aimed at a guess. Fall back
            // to a symmetric one that covers the tool whichever way it points.
            RCLCPP_WARN(get_logger(),
                "Tool axis is not along TCP Y (|dot| = %.2f) — cannot tell tip from "
                "handle, attaching a symmetric %.0f cm box instead.",
                std::abs(toward_tip), 200.0 * tip_len);
            handle_len = tip_len;
            tip_sign = 1.0;
        }
        const double length = handle_len + tip_len;

        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = end_effector_link_;
        aco.object.id = "held_tool";
        aco.object.header.frame_id = end_effector_link_;
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {tool_box_width_m_, length, tool_box_height_m_};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        // Offset the centre so the box reaches tip_len toward the tip and
        // handle_len toward the handle: its centre sits (length/2 - handle_len)
        // from the TCP, on the tip side.
        pose.position.y = tip_sign * (0.5 * length - handle_len);

        aco.object.primitives.push_back(box);
        aco.object.primitive_poses.push_back(pose);
        // Links the box is allowed to touch (the gripper body it is held by).
        aco.touch_links = {
            "gripper_tip_link", "dummy_gripper_link", "wrist_3_link",
            "flange", "tool0",
        };
        psi_.applyAttachedCollisionObject(aco);
        RCLCPP_INFO(get_logger(),
            "Attached held_tool box to %s: %.0f cm toward the tip, %.0f cm toward "
            "the handle.",
            end_effector_link_.c_str(), 100.0 * tip_len, 100.0 * handle_len);
    }

    void detachToolBox() {
        // Two steps, and BOTH are needed. Detaching does not delete the box:
        // MoveIt hands it back to the world as a free-standing object, sitting
        // where the gripper was when it let go. Idempotent: with no tool attached
        // both steps are harmless no-ops.
        moveit_msgs::msg::AttachedCollisionObject aco;
        aco.link_name = end_effector_link_;
        aco.object.id = "held_tool";
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        psi_.applyAttachedCollisionObject(aco);   // synchronous

        // The delete must be synchronous too. removeCollisionObjects() only
        // publishes a /planning_scene diff and returns (says so in its own doc
        // comment), so the next plan may still be checked against a 30 cm ghost
        // box hanging over the tray — and then every motion fails instantly with
        // "start state in collision", holding the tool, with no obvious cause.
        moveit_msgs::msg::CollisionObject rm;
        rm.id = "held_tool";
        rm.header.frame_id = reference_frame_;
        rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        psi_.applyCollisionObject(rm);            // synchronous

        // One service round trip, only ever on a release. Cheap insurance: it
        // turns "maybe there is a ghost box" into a log line.
        for (const auto &name : psi_.getKnownObjectNames()) {
            if (name == "held_tool") {
                RCLCPP_ERROR(get_logger(),
                    "held_tool is STILL in the planning scene after detaching — "
                    "every later plan will be checked against it.");
            }
        }
    }

    // Use only after a grasp check explicitly reports that the tool is no longer
    // held. Stop, clean up the now-stale collision representation, publish a
    // truthful DROPPED event, and plan Home from the fresh live state.
    bool recoverAfterConfirmedToolLoss(
        const std::string &reason,
        std::string &err) {
        RCLCPP_WARN(get_logger(),
            "Confirmed tool-loss recovery (%s): stopping, opening the empty "
            "gripper, then returning home.",
            reason.c_str());
        transporting_.store(false);
        if (move_group_) move_group_->stop();
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        recovery_holds_tool_.store(false);
        detachToolBox();
        publishGripper(true);   // open
        sleepForGripper();
        // The tool fell somewhere we did not intend. The registry must not keep
        // believing the robot holds it — mark it unknown and let perception find
        // it again if it landed on a tray.
        publishHeldToolEvent("DROPPED");
        {   // nothing held any more -> no handover to retrace
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_present_ = false;
        }
        std::string home_err;
        if (!doReturnHomeInternal(home_err)) {
            recovery_error_.store(true);
            publishState("RECOVERY_ERROR", "", "");
            err = "recovery_failed: dropped tool at current pose after " +
                  reason + "; return_home failed: " + home_err;
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            return false;
        }
        recovery_error_.store(false);
        publishState("IDLE", "", "");
        err = "tool_lost: " + reason;
        return true;
    }

    // A ReturnToolHome corridor/placement failure while the gripper still
    // positively holds the tool is not a reason to release it. Freeze safely
    // and keep the planning box attached. return_home may reposition the arm,
    // but only return_tool (or a deliberate release) clears this holding lock.
    void enterHoldingRecovery(
        const std::string &reason,
        const std::string &tool_id,
        const std::string &tool_class,
        std::string &err) {
        transporting_.store(false);
        if (move_group_) move_group_->stop();
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        recovery_holds_tool_.store(true);
        recovery_error_.store(true);
        publishState("RECOVERY_ERROR", tool_id, tool_class);
        err = "holding_recovery: " + reason +
              "; gripper remains closed — use return_home and/or return_tool";
        RCLCPP_ERROR(get_logger(), "%s", err.c_str());
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

    // Discrete handover events for feedback nodes (HRI display, audio). Purely
    // informational — no control-flow effect. Mirrors the legacy loop_mover
    // event names so the existing sound publisher reacts to them too.
    void publishHandoverEvent(const std::string &event) {
        std_msgs::msg::String m;
        m.data = event;
        handover_event_pub_->publish(m);
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

    // Return a fresh tri-state grasp result. In particular, a timeout remains
    // unknown: post-grasp recovery must never translate missing Robotiq data into
    // permission to open the gripper.
    std::optional<bool> freshGripperHoldState() {
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
            return std::nullopt;
        }
        return tool_grasped_value_;
    }

    // Fresh check used by the pre-pick holding guard and normal place logic.
    // Preserve the established pre-pick timeout policy here; safety-critical
    // post-grasp recovery uses freshGripperHoldState() directly instead.
    bool freshGripperHoldsTool() {
        const auto state = freshGripperHoldState();
        if (!state.has_value()) {
            RCLCPP_WARN(get_logger(),
                "Holding-guard re-check timed out; assuming gripper empty.");
            return false;
        }
        return *state;
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
    // The legs of one pick, all planned from the current state before the arm
    // moves at all: reach above the tool (RRT), straight down onto it, straight
    // back up. For a reclaim pick the caller has already parked the arm at the
    // lower reclaim stage pose, so "the current state" is that launch pad.
    moveit_msgs::msg::Constraints instrumentElbowUpConstraints() const {
        moveit_msgs::msg::Constraints constraints;
        constraints.name = "instrument_pick_elbow_up";
        moveit_msgs::msg::JointConstraint elbow;
        elbow.joint_name = "elbow_joint";
        // JointConstraint describes a closed interval around `position`.
        // Centre it between the configured minimum and the UR elbow's +pi hard
        // limit so exactly 0.0 remains valid with the default setting.
        elbow.position = 0.5 * (
            instrument_pick_elbow_min_rad_ +
            tracking_pkg::execution::kUrElbowUpperLimitRad);
        elbow.tolerance_below =
            elbow.position - instrument_pick_elbow_min_rad_;
        elbow.tolerance_above =
            tracking_pkg::execution::kUrElbowUpperLimitRad - elbow.position;
        elbow.weight = 1.0;
        constraints.joint_constraints.push_back(elbow);
        return constraints;
    }

    bool validateElbowUpPlan(
        const moveit::planning_interface::MoveGroupInterface::Plan &plan,
        const std::string &leg,
        double &minimum_out,
        double &final_out,
        std::string &err) const {
        const auto &trajectory = plan.RSN_PLAN_TRAJECTORY.joint_trajectory;
        std::vector<std::vector<double>> samples;
        samples.reserve(trajectory.points.size());
        for (const auto &point : trajectory.points) {
            samples.push_back(point.positions);
        }
        const auto check = tracking_pkg::execution::checkJointMinimum(
            trajectory.joint_names,
            samples,
            "elbow_joint",
            instrument_pick_elbow_min_rad_);
        if (!check.valid) {
            err = "instrument elbow-up guard rejected " + leg + ": " +
                  check.reason;
            if (std::isfinite(check.minimum)) {
                err += " (minimum=" + std::to_string(check.minimum) + " rad)";
            }
            return false;
        }
        minimum_out = check.minimum;
        final_out = check.final;
        return true;
    }

    struct PickLegs {
        moveit::planning_interface::MoveGroupInterface::Plan approach;
        moveit::planning_interface::MoveGroupInterface::Plan descend;
        moveit::planning_interface::MoveGroupInterface::Plan lift;
        moveit::planning_interface::MoveGroupInterface::Plan instrument_stage;
        moveit::planning_interface::MoveGroupInterface::Plan left_stage_pan;
        moveit::planning_interface::MoveGroupInterface::Plan left_stage_transit;
        moveit::planning_interface::MoveGroupInterface::Plan home_transit;
        bool has_return_home_exit = false;
        bool has_home_transit = false;
    };

    // Complete the ReturnToolHome-only pre-flight after the normal 4 cm lift.
    // The first leg reaches the hardware-taught arm posture while preserving
    // wrist_3 from the lift end; its roll around the tool axis is irrelevant at
    // this clearance point and must not cause a separate reset rotation. The
    // second leg rotates only shoulder_pan toward the left side. The final leg
    // reaches the complete hardware-taught Left-Stage TCP pose. For a right-hand
    // slot, an additional full-pose Cartesian leg reaches Home. Every start
    // state is the previous plan's future end state.
    bool tryPlanReturnHomeExit(
        PickLegs &legs,
        bool plan_home_transit,
        std::string &err) {
        std::vector<double> lift_end;
        if (!jointValuesFromPlanEnd(legs.lift, lift_end, err)) {
            err = "lift end state for instrument stage: " + err;
            return false;
        }
        const auto stage_target = tracking_pkg::execution::jointTargetPreserving(
            joint_state_names_, instrument_stage_joints_, lift_end,
            "wrist_3_joint");
        if (!stage_target.has_value()) {
            err = "cannot preserve wrist_3_joint for instrument stage: joint "
                  "names or target sizes are invalid";
            return false;
        }
        const auto wrist3_it = std::find(
            joint_state_names_.begin(), joint_state_names_.end(),
            "wrist_3_joint");
        const size_t wrist3_index = static_cast<size_t>(
            std::distance(joint_state_names_.begin(), wrist3_it));
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        RCLCPP_INFO(get_logger(),
            "ReturnToolHome pre-flight: instrument stage preserves wrist_3 "
            "at %.1f deg (taught value %.1f deg); no redundant tool-roll reset.",
            (*stage_target)[wrist3_index] * kRadToDeg,
            instrument_stage_joints_[wrist3_index] * kRadToDeg);

        move_group_->setStartState(makeStartStateFromPlanEnd(legs.lift));
        if (!planJointPositions(
                *stage_target, legs.instrument_stage, err, false)) {
            err = "instrument stage plan: " + err;
            return false;
        }

        std::vector<double> stage_end;
        if (!jointValuesFromPlanEnd(legs.instrument_stage, stage_end, err)) {
            err = "instrument stage end state: " + err;
            return false;
        }
        const auto pan_it = std::find(
            joint_state_names_.begin(), joint_state_names_.end(),
            "shoulder_pan_joint");
        if (pan_it == joint_state_names_.end()) {
            err = "joint_state_names is missing shoulder_pan_joint";
            return false;
        }
        const size_t pan_index = static_cast<size_t>(
            std::distance(joint_state_names_.begin(), pan_it));
        std::vector<double> left_pan = stage_end;
        left_pan[pan_index] = instrument_left_stage_joints_[pan_index];

        move_group_->setStartState(makeStartStateFromPlanEnd(legs.instrument_stage));
        if (!planJointPositions(
                left_pan, legs.left_stage_pan, err, false)) {
            err = "left-stage shoulder-pan plan: " + err;
            return false;
        }

        geometry_msgs::msg::Pose left_target;
        if (!jointsToTcpPose(instrument_left_stage_joints_, left_target, err)) {
            err = "left-stage target FK: " + err;
            return false;
        }

        // Transit target = the Left-Stage LAUNCH POSITION. For a RIGHT slot we
        // must reach the full taught pose (orientation included), because it is
        // the start point of the home_transit that follows. For a LEFT slot the
        // arm places from here directly, so snapping the wrist to the taught
        // orientation only to rotate it back for the placement is a redundant,
        // visible double-rotation — keep the current (shoulder-pan) orientation
        // instead, a pure translation, and let the local place approach do the
        // one and only wrist rotation.
        geometry_msgs::msg::Pose left_transit_target = left_target;
        if (!plan_home_transit) {
            geometry_msgs::msg::Pose pan_pose;
            if (!jointsToTcpPose(left_pan, pan_pose, err)) {
                err = "left-stage pan FK: " + err;
                return false;
            }
            left_transit_target.orientation = pan_pose.orientation;
        }

        move_group_->setStartState(makeStartStateFromPlanEnd(legs.left_stage_pan));
        std::string linear_err;
        if (!planLinearPose(
                left_transit_target, legs.left_stage_transit, linear_err)) {
            // The fixed Left-Stage pose may need a joint-space solution from the
            // taught shoulder-pan waypoint. It is planned now and cached; nothing
            // is re-planned after the grasp.
            move_group_->setStartState(
                makeStartStateFromPlanEnd(legs.left_stage_pan));
            if (!planPoseTarget(
                    left_transit_target, legs.left_stage_transit, err)) {
                err = "left-stage transit failed (linear: " + linear_err +
                      "; planned: " + err + ")";
                return false;
            }
        }
        legs.has_return_home_exit = true;

        if (plan_home_transit) {
            geometry_msgs::msg::Pose home_target;
            if (!jointsToTcpPose(home_joints_, home_target, err)) {
                err = "home target FK: " + err;
                return false;
            }
            move_group_->setStartState(
                makeStartStateFromPlanEnd(legs.left_stage_transit));
            // Both endpoints are taught full TCP poses. Interpolate position and
            // orientation together; freezing the Left-Stage orientation made IK
            // stop consistently around 35.9% of this corridor.
            constexpr double kCompletePathFraction = 1.0 - 1e-6;
            if (!planLinearPose(
                    home_target, legs.home_transit, err,
                    kCompletePathFraction)) {
                err = "left-stage -> home full-pose plan: " + err;
                return false;
            }
            legs.has_home_transit = true;
        }
        return true;
    }

    bool tryPlanPickSequence(
        const tracking_msgs::msg::GraspCandidate &cand,
        double z_offset_m,
        bool require_elbow_up,
        bool plan_return_home_exit,
        bool plan_return_home_to_home,
        PickLegs &legs,
        geometry_msgs::msg::Pose &approach_pose_out,
        geometry_msgs::msg::Pose &grasp_pose_out,
        std::string &err) {
        legs.has_return_home_exit = false;
        legs.has_home_transit = false;
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
        approach_pose_out.position.z += approach_height_m_;

        const moveit_msgs::msg::Constraints elbow_constraints =
            require_elbow_up ? instrumentElbowUpConstraints()
                             : moveit_msgs::msg::Constraints{};
        const moveit_msgs::msg::Constraints *elbow_constraints_ptr =
            require_elbow_up ? &elbow_constraints : nullptr;
        double approach_min = 0.0, approach_final = 0.0;
        double descend_min = 0.0, descend_final = 0.0;
        double lift_min = 0.0, lift_final = 0.0;

        move_group_->setStartStateToCurrentState();
        if (!planPoseTarget(
                approach_pose_out, legs.approach, err,
                elbow_constraints_ptr)) {
            err = "approach plan: " + err;
            return false;
        }
        if (require_elbow_up && !validateElbowUpPlan(
                legs.approach, "approach", approach_min, approach_final, err)) {
            return false;
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(legs.approach));
        if (!planLinearPose(
                grasp_pose_out, legs.descend, err, -1.0,
                elbow_constraints_ptr)) {
            err = "descend plan: " + err;
            return false;
        }
        if (require_elbow_up && !validateElbowUpPlan(
                legs.descend, "descend", descend_min, descend_final, err)) {
            return false;
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(legs.descend));
        if (!planLinearPose(
                approach_pose_out, legs.lift, err, lift_min_fraction_,
                elbow_constraints_ptr)) {
            err = "lift plan: " + err;
            return false;
        }
        if (require_elbow_up && !validateElbowUpPlan(
                legs.lift, "lift", lift_min, lift_final, err)) {
            return false;
        }
        if (require_elbow_up) {
            const double minimum = std::min(
                approach_min, std::min(descend_min, lift_min));
            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
            RCLCPP_INFO(get_logger(),
                "Instrument elbow-up pre-flight accepted for %s: minimum "
                "elbow %.1f deg; leg endpoints approach=%.1f, descend=%.1f, "
                "lift=%.1f deg (required >= %.1f deg).",
                cand.tool_id.c_str(), minimum * kRadToDeg,
                approach_final * kRadToDeg, descend_final * kRadToDeg,
                lift_final * kRadToDeg,
                instrument_pick_elbow_min_rad_ * kRadToDeg);
        }
        if (plan_return_home_exit && !tryPlanReturnHomeExit(
                legs, plan_return_home_to_home, err)) {
            err = "return-home exit plan: " + err;
            return false;
        }
        return true;
    }

    // ── Grasp core (shared by PickTool and GraspTool) ────────────────

    // Holding-guard → candidate selection → (reclaim only: swing to the lower
    // reclaim pad) → pre-flight (approach + descend + lift planned before motion)
    // → approach → open → descend → close → /tool_grasped check → lift → post-lift
    // verify → escalating local re-grasp → (reclaim only: rise to the upper pad) →
    // attachToolBox().
    //
    // Ends with the tool in the gripper and moves the arm no further. Presenting,
    // handing over and placing are the caller's business.
    //
    // For a reclaim pick the arm is parked on the lower reclaim stage pose before
    // planning, so the approach is planned FROM there — not flown at the tool
    // through the 60 cm camera post — and after the grasp it rises to the upper
    // stage pose before the tool box is attached (down low the box fouls the tray).
    // The instrument tray needs none of this: it is planned directly from home.
    //
    // `chosen_out` is filled as soon as pre-flight commits to a candidate, so a
    // caller can name the tool even when a later motion phase fails.
    bool graspToolCore(const std::string &tool_id_arg,
                       const std::string &location_filter,
                       tracking_msgs::msg::GraspCandidate &chosen_out,
                       geometry_msgs::msg::Pose &grasp_pose_out,
                       geometry_msgs::msg::Pose &approach_pose_out,
                       std::string &err,
                       bool plan_return_home_exit = false,
                       bool plan_return_home_to_home = false) {
        if (recovery_error_.load()) {
            err = "recovery_failed: executor is in RECOVERY_ERROR; "
                  "return_home must succeed before another grasp";
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            return false;
        }
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

        // A new grasp voids any remembered handover pose from a previous one.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_present_ = false;
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

        const bool reclaim = (location_filter == kReclaimLocation);
        const bool require_elbow_up =
            (location_filter == kInstrumentLocation);
        const double z_off = reclaim ? reclaim_z_offset_m_ : z_offset_m_;

        // A reclaim pick is planned from the lower reclaim pad, so get there first.
        // (The instrument tray is planned directly from home — no staging.)
        if (reclaim) {
            // Publish a moving state BEFORE driving to the reclaim tray, so the
            // HRI display turns red during the (long) approach, not only once
            // the arm descends. tool_id_arg is the class for put-back, empty for
            // a class-based pick — either way the display shows red.
            publishState("PICKING", tool_id_arg, "");
            std::string stage_err;
            if (!enterReclaimStaging(stage_err)) {
                err = "reclaim staging: " + stage_err;
                retreatToHome(location_filter);
                return false;
            }
        }

        // Pre-flight: try each candidate in confidence order. Only when every leg
        // plans successfully do we commit to a grasp. No motion happens before this
        // loop succeeds (the reclaim staging above is the one exception — the arm
        // must be at the pad for the approach to plan from the right place).
        //
        // A successful stochastic RRT approach can occasionally end in a robot
        // state from which the Cartesian descent is not available. Reclaim picks
        // therefore retry the COMPLETE sequence from the unchanged lower staging
        // pose. Instrument picks retain their single pre-flight round.
        PickLegs legs;
        geometry_msgs::msg::Pose approach_pose, grasp_pose;
        tracking_msgs::msg::GraspCandidate chosen;
        const int preflight_attempts = reclaim ? reclaim_preflight_attempts_ : 1;
        const auto preflight = tracking_pkg::execution::runPreflightAttempts(
            candidates,
            preflight_attempts,
            [&](const tracking_msgs::msg::GraspCandidate &cand,
                int attempt, std::string &rejection) {
                std::string plan_err;
                if (tryPlanPickSequence(cand, z_off, require_elbow_up,
                                        plan_return_home_exit,
                                        plan_return_home_to_home, legs,
                                        approach_pose, grasp_pose, plan_err)) {
                    chosen = cand;
                    return true;
                }
                rejection = cand.tool_id + " (" + cand.tool_class + "): " + plan_err;
                RCLCPP_WARN(get_logger(),
                    "Pre-flight attempt %d/%d rejected %s: %s",
                    attempt, preflight_attempts,
                    cand.tool_id.c_str(), plan_err.c_str());
                return false;
            },
            [&](int attempt, int total_attempts,
                const std::vector<std::string> &, bool will_retry) {
                if (!reclaim) return;
                if (will_retry) {
                    RCLCPP_WARN(get_logger(),
                        "Reclaim pre-flight attempt %d/%d failed; retrying "
                        "complete pick sequence from lower stage.",
                        attempt, total_attempts);
                    // Discard the constructed start state left by the failed
                    // descend/lift plan. No trajectory is executed here.
                    move_group_->setStartStateToCurrentState();
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Reclaim pre-flight attempt %d/%d failed; no retries "
                        "remaining.", attempt, total_attempts);
                }
            });

        if (!preflight.success) {
            if (reclaim) {
                err = "no reachable reclaim candidate after " +
                      std::to_string(preflight.attempts_run) +
                      " pre-flight attempts. ";
                for (std::size_t attempt = 0;
                     attempt < preflight.rejection_logs.size(); ++attempt) {
                    err += "Attempt " + std::to_string(attempt + 1) + ": ";
                    for (const auto &r : preflight.rejection_logs[attempt]) {
                        err += "[" + r + "] ";
                    }
                }
            } else {
                const auto &rejections = preflight.rejection_logs.front();
                err = "no reachable candidate. Tried " +
                      std::to_string(rejections.size()) + ": ";
                for (const auto &r : rejections) err += "[" + r + "] ";
            }
            retreatToHome(location_filter);
            return false;
        }

        if (reclaim && preflight.attempts_run > 1) {
            RCLCPP_INFO(get_logger(),
                "Reclaim pre-flight attempt %d/%d succeeded from lower stage.",
                preflight.attempts_run, preflight_attempts);
        }

        chosen_out = chosen;
        grasp_pose_out = grasp_pose;
        approach_pose_out = approach_pose;

        RCLCPP_INFO(get_logger(),
            "Pre-flight OK for %s (%s). grasp z=%.4f approach z=%.4f. Executing.",
            chosen.tool_id.c_str(), chosen.tool_class.c_str(),
            grasp_pose.position.z, approach_pose.position.z);

        publishState("PICKING", chosen.tool_id, chosen.tool_class);

        if (!executePlan(legs.approach, err)) {
            err = "approach exec: " + err;
            retreatToHome(location_filter);
            return false;
        }
        publishGripper(true);   // open
        sleepForGripper();

        // Single grasp attempt — no local re-grasp. Two failure modes, both
        // resolved by going home and letting the router re-perceive and retry
        // the class rather than stabbing at the tray again from a guessed pose:
        //  - gripper closes on nothing (empty at close): the tool isn't where
        //    we expected  -> grasp_failed.
        //  - tool grasped but slips while lifting                 -> tool_lost.
        const geometry_msgs::msg::Pose secured_grasp_pose = grasp_pose;

        if (!executePlan(legs.descend, err)) { err = "descend exec: " + err; return false; }
        publishGripper(false);  // close
        if (!waitForFreshToolGrasped(grasp_check_timeout_sec_)) {
            // Empty at close -> re-perceive instead of re-grasping blindly.
            RCLCPP_WARN(get_logger(),
                "Grasp check failed (gripper empty at close). Returning home.");
            publishGripper(true);
            std::string lift_err;
            executePlan(legs.lift, lift_err);   // raise empty gripper off the tray
            retreatToHome(location_filter);
            err = "grasp_failed: no tool in gripper after close";
            return false;
        }

        // ReturnToolHome owns a deterministic post-grasp recovery. Remember the
        // physical pick as soon as the close check succeeds, so any later lift or
        // cached-exit execution failure can truthfully emit DROPPED before opening
        // at the current pose. Generic picks retain their historical event timing.
        if (plan_return_home_exit) {
            rememberHeldTool(chosen, secured_grasp_pose);
            publishHeldToolEvent("PICKED");
        }

        // Grasped -> lift, briefly settle, then force a fresh check. The settle
        // lets a marginal grip relax before we commit, and the fresh check
        // avoids the async monitor's ~0.5 s lag.
        if (!executePlan(legs.lift, err)) {
            err = (plan_return_home_exit ? "post_grasp: " : "") +
                  std::string("lift exec: ") + err;
            return false;
        }
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, post_lift_settle_sec_))));
        if (!verifyGraspAfterLift()) {
            if (plan_return_home_exit) {
                err = "post_grasp: tool_lost during lift verification";
                return false;
            }
            // Slipped during the lift -> home, let the router retry the class.
            RCLCPP_WARN(get_logger(), "Tool slipped during lift. Returning home.");
            publishGripper(true);
            retreatToHome(location_filter);   // best-effort: free the tray camera
            err = "tool_lost: tool slipped during lift";
            return false;
        }

        // The tool is in the gripper. Record where it came from BEFORE moving it,
        // so that any failure from here on can still be undone with return_tool —
        // putting it back where it was found is the only safe way out of a failure
        // while holding something. This is the pose the jaws actually closed on;
        // the caller's grasp_pose_out follows suit — return_tool_home derives its
        // grip depth d from it.
        grasp_pose_out = secured_grasp_pose;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_pick_grasp_pose_ = secured_grasp_pose;
            last_pick_approach_pose_ = approach_pose;
            last_pick_location_ = location_filter;
            have_last_pick_ = true;
        }

        // ReturnToolHome executes the already-validated exit exactly as planned:
        // lift -> fixed arm posture with preserved wrist_3 -> controlled
        // full-pose left-stage
        // transit -> (right slot only) full-pose Home transit. No held_tool box
        // exists on this hardware-validated corridor, and no plan is recomputed
        // after the gripper closes.
        if (reclaim && plan_return_home_exit) {
            if (!legs.has_return_home_exit) {
                err = "post_grasp: cached ReturnToolHome exit is missing";
                return false;
            }
            publishState("TRANSPORTING", chosen.tool_id, chosen.tool_class);
            transporting_.store(true);
            struct ExitTransportGuard {
                std::atomic<bool> &flag;
                ~ExitTransportGuard() { flag.store(false); }
            } exit_transport_guard{transporting_};

            std::string exit_err;
            for (const auto phase :
                 tracking_pkg::execution::returnHomePostLiftPhases()) {
                const moveit::planning_interface::MoveGroupInterface::Plan *plan =
                    nullptr;
                std::string label;
                switch (phase) {
                    case tracking_pkg::execution::ReturnHomeExitPhase::
                            INSTRUMENT_STAGE:
                        plan = &legs.instrument_stage;
                        label = "instrument stage";
                        RCLCPP_INFO(get_logger(),
                            "ReturnToolHome cached exit: lift -> "
                            "instrument stage (wrist_3 preserved).");
                        break;
                    case tracking_pkg::execution::ReturnHomeExitPhase::
                            LEFT_STAGE_PAN:
                        plan = &legs.left_stage_pan;
                        label = "left-stage shoulder pan";
                        RCLCPP_INFO(get_logger(),
                            "ReturnToolHome cached exit: instrument stage -> "
                            "left-stage shoulder pan.");
                        break;
                    case tracking_pkg::execution::ReturnHomeExitPhase::
                            LEFT_STAGE_TRANSIT:
                        plan = &legs.left_stage_transit;
                        label = "left-stage full-pose transit";
                        RCLCPP_INFO(get_logger(),
                            "ReturnToolHome cached exit: shoulder pan -> "
                            "left-stage full-pose transit.");
                        break;
                }
                if (!plan || !executePlan(*plan, exit_err)) {
                    err = "post_grasp: " + label + " exec: " + exit_err;
                    return false;
                }
            }
            if (plan_return_home_to_home) {
                if (!legs.has_home_transit) {
                    err = "post_grasp: cached Left-Stage -> Home transit is missing";
                    return false;
                }
                RCLCPP_INFO(get_logger(),
                    "ReturnToolHome cached exit: left stage -> home full-pose "
                    "Cartesian transit.");
                if (!executePlan(legs.home_transit, exit_err)) {
                    err = "post_grasp: cached left-stage -> home exec: " + exit_err;
                    return false;
                }
            }
            if (!freshGripperHoldsTool()) {
                err = "post_grasp: tool_lost on cached exit to " +
                      std::string(plan_return_home_to_home ? "home" : "left stage");
                return false;
            }

            // Only now is the real tool represented in the planning scene. The
            // subsequent local slot plans retain their full collision checking.
            attachToolBox(chosen, grasp_pose);
            return true;
        }

        // For a generic reclaim pick, rise to the upper stage pose before attaching the
        // box: the box reaches tool_box_tip_m_ (0.18 m) toward the tip, and down in
        // the tray that would sweep the camera post and wedge every later plan. The
        // upper pad is 213 mm above the tray plane — clear. The instrument tray's
        // approach pose is already free space, so nothing to do there.
        if (reclaim) {
            std::string ex_err;
            if (!exitReclaimToUpper(ex_err)) {
                err = "reclaim exit to upper: " + ex_err;
                return false;
            }
        }

        // Tool is clear of the tray: make it visible to the planner so any onward
        // motion (present rotation, handover, place-back) avoids the tray-camera
        // stand and accounts for the tool's reach beyond the gripper.
        attachToolBox(chosen, grasp_pose);

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
        const bool grasped = graspToolCore(
            tool_id_arg, kInstrumentLocation,
            chosen, grasp_pose, approach_pose, err);
        // chosen is filled the moment pre-flight commits, so the tool can still
        // be named when a later motion phase fails — the LLM keys its retry on
        // the class.
        picked_id_out = chosen.tool_id;
        picked_class_out = chosen.tool_class;
        if (!grasped) return false;
        // graspToolCore already recorded the pick, the moment the tool was secured.

        publishState("TRANSPORTING", chosen.tool_id, chosen.tool_class);
        // From here the tool is in transit: a /tool_grasped=false stops the
        // current motion immediately (see toolGraspedCb). After each move we
        // check whether that happened and bail to home if so.
        transporting_.store(true);

        // A tool picked on the RIGHT side goes to home first: from there the present
        // rotation is clear. A tool picked on the LEFT turns toward the surgeon
        // straight away — no detour through home, one less stop.
        if (grasp_pose.position.x > instrument_right_side_x_) {
            std::string home_err;
            if (!moveToJointPositions(home_joints_, home_err)) {
                bool hd = false;
                if (!lastToolGrasped(hd) && hd) {
                    transporting_.store(false);
                    recoverAfterConfirmedToolLoss(
                        "tool dropped returning home before handover", err);
                    return false;
                }
                transporting_.store(false);
                err = "return home before handover: " + home_err;
                return false;
            }
        }

        // Turn the arm around to present the tool. The attached tool box + the
        // tray-camera stand in the scene keep this collision-free.
        const bool present_ok = rotateShoulderPanTo(present_shoulder_pan_rad_, err);
        {
            bool hd = false;
            if (!lastToolGrasped(hd) && hd) {
                transporting_.store(false);
                recoverAfterConfirmedToolLoss(
                    "tool dropped during present rotation", err);
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
                recoverAfterConfirmedToolLoss(
                    "tool dropped during pre-orient", err);
                return false;
            }
        }
        if (!preorient_ok) {
            transporting_.store(false);
            err = "pre-orient wrists: " + err;
            return false;
        }

        // This IS the "waiting for handoff" pose. Remember its joints so a
        // later return_tool can retrace the handover through it instead of
        // free-planning home from wherever the arm ended up.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_present_joints_ = move_group_->getCurrentJointValues();
            have_last_present_ = !last_present_joints_.empty();
        }

        transporting_.store(false);
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
            // Distinct state so the HRI display shows amber "make your gesture"
            // instead of the stale TRANSPORTING (red) from the pick. Cleared
            // implicitly by the HANDOVER/PRESENTING states after the gesture.
            {
                std::string tid, tcls;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    tid = active_tool_id_;
                    tcls = active_tool_class_;
                }
                publishState("AWAIT_GESTURE", tid, tcls);
            }
            while (rclcpp::ok()) {
                publishHandoverFeedback(goal_handle, "AWAITING_GESTURE");
                if (!waitForGesture(goal_handle)) {
                    // Tool fell out while waiting? Treat as a loss and retry.
                    bool have_grasp_data = false;
                    if (!lastToolGrasped(have_grasp_data) && have_grasp_data) {
                        recoverAfterConfirmedToolLoss(
                            "gripper empty during handover", err);
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
                    // Tell the surgeon (display flashes red, audio buzzes).
                    publishHandoverEvent("reachability:unreachable");
                    continue;
                }
                RCLCPP_INFO(get_logger(), "Reachability decision: reachable.");
                publishHandoverEvent("gesture_detected");
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
            recoverAfterConfirmedToolLoss(
                "gripper empty before handover", err);
            return false;
        }

        publishState("HANDOVER", tool_id_snapshot, tool_class_snapshot);
        publishHandoverFeedback(goal_handle, "MOVING_TO_HAND");
        if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            err = "handover execute failed";
            return false;
        }

        // The arm has reached the hand and is stationary: show green "take it"
        // immediately. Keep the dwell below before enabling the physical release.
        publishState("PRESENTING", tool_id_snapshot, tool_class_snapshot);

        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.0, pre_release_dwell_seconds_))));

        // The tool remained still for the configured dwell; the force-guided
        // release may now wait for the surgeon to pull it.
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
        {   // handover complete -> nothing to retrace any more
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_present_ = false;
        }

        // Tool delivered — drop the attached collision box and forget the pick.
        detachToolBox();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_pick_ = false;
        }

        if (return_home_after_handover_) {
            std::string home_err;
            if (!doReturnHomeInternal(home_err)) {
                recovery_error_.store(true);
                publishState("RECOVERY_ERROR", "", "");
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

    // Reach the complete taught Left-Stage pose through the same controlled
    // shoulder-pan-first transition as the cached ReturnToolHome exit. The
    // gripper is empty on the reclaim/handover returns that use this helper.
    bool moveToInstrumentLeftStage(std::string &err) {
        RCLCPP_INFO(get_logger(),
            "Home return via Left-Stage: controlled shoulder-pan transition.");
        if (!rotateShoulderPanTo(instrument_left_stage_joints_[0], err)) {
            err = "left-stage shoulder pan: " + err;
            return false;
        }

        geometry_msgs::msg::Pose left_target;
        if (!jointsToTcpPose(
                instrument_left_stage_joints_, left_target, err)) {
            err = "left-stage target FK: " + err;
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->setStartStateToCurrentState();
        std::string linear_err;
        if (planLinearPose(left_target, plan, linear_err) &&
                executePlan(plan, linear_err)) {
            return true;
        }
        RCLCPP_WARN(get_logger(),
            "Home return: linear Left-Stage transit not clean (%s); using "
            "the same full-pose planned fallback as ReturnToolHome.",
            linear_err.c_str());
        if (!moveToPoseTarget(left_target, err)) {
            err = "left-stage full-pose transit: " + err;
            return false;
        }
        return true;
    }

    // The final Left-Stage -> Home leg is shared by empty reclaim returns and
    // the tool-carrying ReturnToolHome pre-flight.
    // Require the complete full-pose Cartesian path and never replace it with a
    // stochastic detour.
    bool moveLeftStageToHome(std::string &err) {
        geometry_msgs::msg::Pose home_target;
        if (!jointsToTcpPose(home_joints_, home_target, err)) {
            err = "home target FK: " + err;
            return false;
        }
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        move_group_->setStartStateToCurrentState();
        constexpr double kCompletePathFraction = 1.0 - 1e-6;
        if (!planLinearPose(
                home_target, home_plan, err, kCompletePathFraction)) {
            err = "left-stage -> home full-pose plan: " + err;
            return false;
        }
        RCLCPP_INFO(get_logger(),
            "Home return via Left-Stage: full-pose Cartesian transit to Home.");
        if (!executePlan(home_plan, err)) {
            err = "left-stage -> home execute: " + err;
            return false;
        }
        return true;
    }

    bool doReturnHomeInternal(
        std::string &err,
        HomeReturnOrigin origin = HomeReturnOrigin::DIRECT) {
        publishState("RETURNING", "", "");
        if (home_joints_.size() != 6) {
            err = "home_joints must have 6 values";
            return false;
        }

        if (tracking_pkg::execution::homeReturnUsesInstrumentStage(origin)) {
            RCLCPP_INFO(get_logger(),
                "Reclaim -> Home empty return: reclaim -> instrument stage.");
            if (!moveToJointPositions(instrument_stage_joints_, err)) {
                err = "reclaim -> instrument stage: " + err;
                return false;
            }
        }

        if (tracking_pkg::execution::homeReturnUsesLeftStage(origin)) {
            if (!moveToInstrumentLeftStage(err)) return false;
            return moveLeftStageToHome(err);
        }
        return moveToJointPositions(home_joints_, err);
    }

    // ── Place primitive (shared by ReturnTool and ReturnToolHome) ────

    // Put the held tool down at release_pose on `location`'s tray and let go.
    //
    // Instrument tray: reach the slot's side through a taught transit pose first,
    // then plan the short local approach from there (approach RRT -> descend ->
    // release -> lift). A Reclaim->RIGHT ReturnToolHome arrives here only after
    // its cached full-pose Left-Stage->Home leg; Reclaim->LEFT arrives at
    // Left-Stage. Other callers keep their existing transit behavior.
    //
    // Reclaim tray (used by normal return_tool): stage through the two reclaim
    // pads exactly like a reclaim grasp —
    // upper → lower, descend/release from the lower pad, then rise back to upper to
    // clear the camera post.
    enum class PlaceRouteContext {
        DEFAULT,
        RETURN_HOME_AT_LEFT_STAGE,
        RETURN_HOME_AT_HOME,
    };

    bool placeToolAt(const std::string &location,
                     const geometry_msgs::msg::Pose &release_pose,
                     std::string &err,
                     PlaceRouteContext route_context = PlaceRouteContext::DEFAULT) {
        const bool reclaim = (location == kReclaimLocation);

        // A tool can slip out anywhere on the way to the tray. Arm transporting_
        // so a /tool_grasped=false stops the running motion at once (toolGraspedCb),
        // and clear it on every exit. A motion that failed WHILE the gripper went
        // empty is retagged tool_lost so the caller marks the tool DROPPED instead
        // of trying to place nothing; a motion that failed while still holding is a
        // genuine planning/exec failure and keeps its message.
        transporting_.store(true);
        struct TransportGuard {
            std::atomic<bool> &flag;
            ~TransportGuard() { flag.store(false); }
        } transport_guard{transporting_};
        auto exec_or_lost = [&](moveit::planning_interface::MoveGroupInterface::Plan
                                    &plan, const std::string &where) {
            if (executePlan(plan, err)) return true;
            err = freshGripperHoldsTool()
                      ? (where + ": " + err)
                      : ("tool_lost: gripper empty during " + where);
            return false;
        };

        // Get the arm to the launch point the place is planned from.
        if (reclaim) {
            std::string stage_err;
            if (!enterReclaimStaging(stage_err)) {
                err = freshGripperHoldsTool()
                          ? ("place reclaim staging: " + stage_err)
                          : "tool_lost: gripper empty during reclaim staging";
                return false;
            }
        } else {
            const bool right = release_pose.position.x > instrument_right_side_x_;
            const bool already_at_left_stage =
                route_context == PlaceRouteContext::RETURN_HOME_AT_LEFT_STAGE;
            const bool already_at_home =
                route_context == PlaceRouteContext::RETURN_HOME_AT_HOME;
            // Generic placement keeps the held tool's orientation across its
            // transit. ReturnToolHome instead arrives through its cached route:
            // a LEFT slot from Left-Stage, a RIGHT slot from Home.
            if (right && !already_at_left_stage && !already_at_home) {
                std::string via_err;
                if (!moveAcrossKeepingOrientation(
                        instrument_left_stage_joints_, via_err)) {
                    // Best-effort intermediate: if it will not plan, still try to
                    // reach home directly rather than aborting the place.
                    RCLCPP_WARN(get_logger(),
                        "Place route via left stage not clean (%s) — going to "
                        "home directly.", via_err.c_str());
                }
            }
            if (already_at_home) {
                RCLCPP_INFO(get_logger(),
                    "ReturnToolHome route: cached full-pose transit already "
                    "reached Home; planning local right-slot approach.");
            } else if (already_at_left_stage) {
                if (right) {
                    err = "internal ReturnToolHome route mismatch: right slot "
                          "did not pre-plan the cached Home transit";
                    return false;
                }
                RCLCPP_INFO(get_logger(),
                    "ReturnToolHome route: already at left stage; planning local "
                    "left-slot approach.");
            } else {
                const std::vector<double> &transit =
                    right ? home_joints_ : instrument_left_stage_joints_;
                std::string transit_err;
                const bool transit_ok =
                    moveAcrossKeepingOrientation(transit, transit_err);
                if (!transit_ok) {
                    err = freshGripperHoldsTool()
                              ? (std::string("place route via ") +
                                 (right ? "home" : "left stage") + ": " + transit_err)
                              : "tool_lost: gripper empty during place transit";
                    return false;
                }
            }
        }

        // Reach the placement orientation well ABOVE the slot, then descend
        // straight down: the approach plan may rotate the tool to its final
        // orientation, but it does so high up; the descend below is a pure
        // vertical cartesian move that never rotates. The tight reclaim tray
        // keeps its small approach height (its staging handles clearance).
        geometry_msgs::msg::Pose approach_pose = release_pose;
        approach_pose.position.z +=
            reclaim ? approach_height_m_ : place_rotate_height_m_;

        moveit::planning_interface::MoveGroupInterface::Plan
            approach_plan, descend_plan, lift_plan;
        move_group_->setStartStateToCurrentState();
        if (!planPoseTarget(approach_pose, approach_plan, err)) {
            err = "place approach plan: " + err; return false;
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(approach_plan));
        std::string descend_err;
        if (!planLinearPose(release_pose, descend_plan, descend_err)) {
            // The straight cartesian descend is blocked (e.g. the big held_tool
            // box grazes a tray rail near the slot), even though the release pose
            // itself is reachable — the tool lay there at registration. Fall back
            // to a full motion plan to the same pose: RRT can route around the
            // graze. planPoseTarget already retries (plan_attempts).
            RCLCPP_WARN(get_logger(),
                "Place descend cartesian low (%s) — falling back to a planned "
                "descend to the release pose.", descend_err.c_str());
            move_group_->setStartState(makeStartStateFromPlanEnd(approach_plan));
            if (!planPoseTarget(release_pose, descend_plan, err)) {
                err = "place descend plan (cartesian and planned both failed): "
                      + err;
                return false;
            }
        }
        move_group_->setStartState(makeStartStateFromPlanEnd(descend_plan));
        std::string lift_err;
        if (!planLinearPose(approach_pose, lift_plan, lift_err)) {
            RCLCPP_WARN(get_logger(),
                "Place lift cartesian low (%s) — falling back to a planned lift.",
                lift_err.c_str());
            move_group_->setStartState(makeStartStateFromPlanEnd(descend_plan));
            if (!planPoseTarget(approach_pose, lift_plan, err)) {
                err = "place lift plan (cartesian and planned both failed): " + err;
                return false;
            }
        }

        RCLCPP_INFO(get_logger(), "Placing at (%.3f, %.3f, %.3f) on the %s tray.",
            release_pose.position.x, release_pose.position.y,
            release_pose.position.z, location.c_str());

        if (!exec_or_lost(approach_plan, "place approach exec")) return false;
        if (!exec_or_lost(descend_plan,  "place descend exec"))  return false;

        // Last check before we let go: if the tool is already gone, do not open
        // on an empty gripper and claim a place — report the loss. Clear the
        // transport flag first so the intentional open below does not trip the
        // loss monitor and stop the lift.
        transporting_.store(false);
        if (!freshGripperHoldsTool()) {
            err = "tool_lost: gripper empty before release";
            return false;
        }
        publishGripper(true);  // open — release the tool
        sleepForGripper();
        detachToolBox();
        if (!executePlan(lift_plan, err))     { err = "place lift exec: "     + err; return false; }

        // Rise out of the reclaim tray before anything else moves — the box is
        // detached now, but the gripper itself must still clear the camera post.
        if (reclaim) {
            std::string ex_err;
            if (!exitReclaimToUpper(ex_err)) {
                err = "placed, but could not clear the reclaim post: " + ex_err;
                return false;
            }
        }
        return true;
    }

    // ── Skill: ReturnTool ────────────────────────────────────────────

    // Brings the currently-held tool back to where it was picked from and
    // releases it return_release_height_m_ above the original grasp pose.
    bool doReturnTool(std::string &err) {
        geometry_msgs::msg::Pose grasp_pose;
        std::string tool_id_snapshot, tool_class_snapshot, location;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_last_pick_) {
                err = "no recorded pick to return";
                return false;
            }
            grasp_pose = last_pick_grasp_pose_;
            location = last_pick_location_;
            tool_id_snapshot = active_tool_id_;
            tool_class_snapshot = active_tool_class_;
        }

        publishState("RETURNING", tool_id_snapshot, tool_class_snapshot);

        // First pull back to the present ("waiting for handoff") pose. From the
        // surgeon's hand the arm is stretched out into the room, and a path
        // free-planned from there swings erratically; retracing through the
        // present pose keeps the return to a base rotation plus the local
        // approach — the same corridor as the way out. Best-effort: if the
        // retract will not plan, the place transit below still has retries.
        std::vector<double> present;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (have_last_present_) present = last_present_joints_;
            have_last_present_ = false;
        }
        if (!present.empty()) {
            std::string via_err;
            if (!moveToJointPositions(present, via_err)) {
                RCLCPP_WARN(get_logger(),
                    "Return via present pose not clean (%s) — continuing from "
                    "the current pose.", via_err.c_str());
            }
        }

        geometry_msgs::msg::Pose release_pose = grasp_pose;
        release_pose.position.z += return_release_height_m_;

        // Back down the same tray it came from, so the corridor is the right one.
        if (!placeToolAt(location, release_pose, err)) {
            // The tool fell out on the way back: mark it DROPPED (registry ->
            // UNKNOWN) and go home, rather than reporting it neatly placed.
            if (err.rfind("tool_lost", 0) == 0) {
                std::string lost_err;
                recoverAfterConfirmedToolLoss(
                    "return_tool: " + err, lost_err);
            }
            return false;
        }

        // Back where it was picked from — for the instrument tray, that is its home.
        publishHeldToolEvent("PLACED_HOME");

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            have_last_pick_ = false;
        }

        std::string home_err;
        const HomeReturnOrigin home_origin = location == kReclaimLocation
            ? HomeReturnOrigin::RECLAIM
            : HomeReturnOrigin::DIRECT;
        if (!doReturnHomeInternal(home_err, home_origin)) {
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
                          geometry_msgs::msg::Pose &release_out) {
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
    }

    // The goal names a tool — but WHICH kind of name? A track id (reclaim_0), a
    // tool class (hammer), or a registry slot id (hammer_1).
    //
    // Track ids are EPHEMERAL: the world model evicts a tool after ~10 s of
    // occlusion and re-mints a fresh id when it reappears. So insisting on one
    // would be fragile even for a caller that gets it right — by the time we
    // grasp, reclaim_0 may already be reclaim_3. Match leniently, and let the
    // CLASS be the durable name.
    static std::string classGuessFromName(std::string q) {
        for (const std::string &p : {std::string("tool_"), std::string("reclaim_")}) {
            if (q.rfind(p, 0) == 0) { q = q.substr(p.size()); break; }
        }
        // A registry slot id: strip a trailing "_<digits>" ("hammer_1" -> "hammer").
        const auto us = q.rfind('_');
        if (us != std::string::npos && us + 1 < q.size()) {
            bool digits = true;
            for (size_t i = us + 1; i < q.size(); ++i) {
                if (!std::isdigit(static_cast<unsigned char>(q[i]))) { digits = false; break; }
            }
            if (digits) q = q.substr(0, us);
        }
        return q;
    }

    static bool nameMatches(const tracking_msgs::msg::GraspCandidate &c,
                            const std::string &q) {
        if (q.empty()) return true;
        return c.tool_id == q
            || c.tool_class == q
            || c.tool_class == classGuessFromName(q);
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
            std::string on_tray;
            for (const auto &c : candidates) {
                on_tray += (on_tray.empty() ? "" : ", ") + c.tool_class;
            }
            candidates.erase(
                std::remove_if(candidates.begin(), candidates.end(),
                    [&](const auto &c) { return !nameMatches(c, tool_id_arg); }),
                candidates.end());
            if (candidates.empty()) {
                err = "'" + tool_id_arg + "' is not on the reclaim tray (there: "
                      + (on_tray.empty() ? "nothing" : on_tray) + ")";
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

            // The side is already known from the untouched candidate: grasp x/y
            // is exactly the point later returned by graspToolCore. For a right
            // slot, extend the pre-flight through the complete fixed-pose
            // Left-Stage -> Home leg before allowing the gripper to close.
            const double planned_grasp_distance = std::hypot(
                cand.grasp_pose.pose.position.x - cand.handle_center.x,
                cand.grasp_pose.pose.position.y - cand.handle_center.y);
            geometry_msgs::msg::Pose planned_release_pose;
            computePlacePose(home, planned_grasp_distance, planned_release_pose);
            const bool preplan_to_home =
                tracking_pkg::execution::returnHomeNeedsHomeTransit(
                    planned_release_pose.position.x,
                    instrument_right_side_x_);

            tracking_msgs::msg::GraspCandidate chosen;
            geometry_msgs::msg::Pose grasp_pose, approach_pose;
            std::string grasp_err;
            if (!graspToolCore(cand.tool_id, kReclaimLocation,
                               chosen, grasp_pose, approach_pose, grasp_err,
                               true, preplan_to_home)) {
                RCLCPP_WARN(get_logger(), "Could not grasp %s: %s",
                            cand.tool_id.c_str(), grasp_err.c_str());
                if (recovery_error_.load()) {
                    err = "recovery_failed: could not return Home after "
                          "pre-grasp failure for " + cand.tool_id + ": " +
                          grasp_err;
                    return false;
                }
                if (grasp_err.rfind("post_grasp:", 0) == 0) {
                    const auto still_held = freshGripperHoldState();
                    if (!still_held.has_value()) {
                        RCLCPP_WARN(get_logger(),
                            "ReturnToolHome post-grasp check timed out; keeping "
                            "the gripper closed conservatively.");
                    }
                    if (tracking_pkg::execution::postGraspFailureDisposition(
                            still_held) == tracking_pkg::execution::
                                PostGraspFailureDisposition::KEEP_HOLDING) {
                        // The cached corridor failed, but the tool did not. Keep
                        // it instead of opening at an arbitrary point.
                        attachToolBox(chosen, grasp_pose);
                        enterHoldingRecovery(
                            "return_tool_home: " + grasp_err,
                            chosen.tool_id, chosen.tool_class, err);
                    } else {
                        std::string recovery_err;
                        recoverAfterConfirmedToolLoss(
                            "return_tool_home: " + grasp_err, recovery_err);
                        err = recovery_err;
                    }
                    return false;
                }
                skipped_ids_out.push_back(cand.tool_id);
                skipped_reasons_out.push_back(grasp_err);
                // A pre-grasp failure leaves the tool untouched and
                // graspToolCore already retreats the empty gripper Home.
                continue;
            }

            // Where along the tool are we holding it? THIS is what the place pose
            // has to be built from — not the pose we grasped at.
            const double d = std::hypot(
                grasp_pose.position.x - chosen.handle_center.x,
                grasp_pose.position.y - chosen.handle_center.y);

            geometry_msgs::msg::Pose release_pose;
            computePlacePose(home, d, release_pose);

            publishState("RETURNING", chosen.tool_id, chosen.tool_class);
            RCLCPP_INFO(get_logger(),
                "ReturnToolHome: %s (%s) -> slot %s. Holding it %.1f mm along its "
                "shaft, so releasing at (%.3f, %.3f, %.3f).",
                chosen.tool_id.c_str(), chosen.tool_class.c_str(),
                home.slot_id.c_str(), d * 1000.0,
                release_pose.position.x, release_pose.position.y,
                release_pose.position.z);

            // graspToolCore has already executed the complete cached corridor:
            // right slots end at Home, left slots at Left-Stage. Only the local
            // slot approach is planned after the grasp.
            std::string place_err;
            if (!placeToolAt(
                    kInstrumentLocation,
                    release_pose,
                    place_err,
                    preplan_to_home
                        ? PlaceRouteContext::RETURN_HOME_AT_HOME
                        : PlaceRouteContext::RETURN_HOME_AT_LEFT_STAGE)) {
                const auto still_held = freshGripperHoldState();
                if (!still_held.has_value()) {
                    RCLCPP_WARN(get_logger(),
                        "ReturnToolHome placement grasp check timed out; keeping "
                        "the gripper closed conservatively.");
                }
                if (tracking_pkg::execution::postGraspFailureDisposition(
                        still_held) == tracking_pkg::execution::
                            PostGraspFailureDisposition::KEEP_HOLDING) {
                    enterHoldingRecovery(
                        "return_tool_home placement: " + place_err,
                        chosen.tool_id, chosen.tool_class, err);
                } else {
                    std::string recovery_err;
                    recoverAfterConfirmedToolLoss(
                        "return_tool_home placement: " + place_err,
                        recovery_err);
                    err = recovery_err;
                }
                return false;
            }

            publishHeldToolEvent("PLACED_HOME");
            returned_out.push_back(home.slot_id);

            std::string home_ret;
            if (!doReturnHomeInternal(home_ret)) {
                recovery_error_.store(true);
                publishState("RECOVERY_ERROR", "", "");
                err = "recovery_failed: tool was placed in " + home.slot_id +
                      " but return_home failed: " + home_ret;
                RCLCPP_ERROR(get_logger(), "%s", err.c_str());
                return false;
            }
            recovery_error_.store(false);
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
        if (recovery_error_.load()) {
            RCLCPP_ERROR(get_logger(),
                "PickTool rejected: executor is in RECOVERY_ERROR; run "
                "return_home successfully first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
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
            publishIdleUnlessRecoveryError();
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
        if (recovery_error_.load()) {
            RCLCPP_ERROR(get_logger(),
                "GraspTool rejected: executor is in RECOVERY_ERROR; run "
                "return_home successfully first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
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

        // graspToolCore stages the reclaim tray itself (upper/lower pads); the
        // instrument tray is planned directly from home.
        tracking_msgs::msg::GraspCandidate chosen;
        geometry_msgs::msg::Pose grasp_pose, approach_pose;
        std::string err;
        const bool ok = graspToolCore(goal->tool_id, location, chosen,
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
            publishIdleUnlessRecoveryError();
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
        if (recovery_error_.load()) {
            RCLCPP_ERROR(get_logger(),
                "ReturnToolHome rejected: executor is in RECOVERY_ERROR; run "
                "return_home successfully first.");
            return rclcpp_action::GoalResponse::REJECT;
        }
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
        publishIdleUnlessRecoveryError();
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
        publishIdleUnlessRecoveryError();
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
        if (ok) {
            recovery_holds_tool_.store(false);
            recovery_error_.store(false);
        }
        publishIdleUnlessRecoveryError();
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
        std::string err, held_id, held_class;
        const bool holding_recovery = recovery_holds_tool_.load();
        if (holding_recovery) {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            held_id = active_tool_id_;
            held_class = active_tool_class_;
        }
        const bool ok = doReturnHomeInternal(err);
        result->success = ok;
        result->message = ok
            ? (holding_recovery
                   ? "home reached; tool remains held — use return_tool"
                   : "ok")
            : err;
        if (ok) {
            if (holding_recovery) {
                recovery_error_.store(true);
                publishState("RECOVERY_ERROR", held_id, held_class);
                RCLCPP_WARN(get_logger(),
                    "Home reached, but recovery tool remains held. IDLE is "
                    "blocked until return_tool or release_tool succeeds.");
            } else {
                recovery_error_.store(false);
                publishState("IDLE", "", "");
            }
        } else {
            recovery_error_.store(true);
            publishState("RECOVERY_ERROR", "", "");
            RCLCPP_ERROR(get_logger(),
                "return_home failed; executor remains in RECOVERY_ERROR: %s",
                err.c_str());
        }
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
        if (ok) {
            recovery_holds_tool_.store(false);
            recovery_error_.store(false);
        }
        publishIdleUnlessRecoveryError();
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
    double place_rotate_height_m_;
    double tool_yaw_offset_rad_;
    double instrument_pick_elbow_min_rad_;
    double tool_box_handle_m_;
    double tool_box_tip_m_;
    double tool_box_width_m_;
    double tool_box_height_m_;
    int plan_attempts_;
    int reclaim_preflight_attempts_;
    double reclaim_exit_clearance_m_;
    std::vector<double> instrument_stage_joints_;
    std::vector<double> instrument_left_stage_joints_;
    std::vector<double> reclaim_stage_upper_joints_;
    std::vector<double> reclaim_stage_lower_joints_;
    double instrument_right_side_x_;
    std::vector<std::string> reversed_tool_box_classes_;
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
    double cartesian_min_fraction_;
    double lift_min_fraction_;
    double gesture_wait_timeout_sec_;
    double post_gesture_settle_sec_;
    double return_release_height_m_;
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr handover_event_pub_;
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
    // Blocks pick-like goals after an incomplete recovery. In the stronger
    // recovery_holds_tool_ case, reaching Home alone deliberately does not clear
    // this flag because the gripper still contains a tool.
    std::atomic<bool> recovery_error_{false};
    // Stronger recovery state: the gripper intentionally remains closed around
    // a tool. Reaching Home alone must not publish IDLE or clear the pick lock.
    std::atomic<bool> recovery_holds_tool_{false};
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
    // The exact joints of the "waiting for handoff" pose. return_tool retraces
    // the handover through this pose so the way back stays as tame as the way
    // out (base rotation + local approach). Guarded by state_mutex_.
    std::vector<double> last_present_joints_;
    bool have_last_present_ = false;
    // Which tray it came from — return_tool has to walk that tray's corridor.
    std::string last_pick_location_;
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

    // Spin in the background so the startup home move can read the current state
    // and plan. Then drive to home once before handing control to the executor.
    std::thread spin_thread([&executor]() { executor.spin(); });
    node->goHomeOnStartup();
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
