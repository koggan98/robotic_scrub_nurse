#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace tracking_pkg
{
namespace execution
{

enum class ReturnHomeExitPhase
{
  INSTRUMENT_STAGE,
  LEFT_STAGE_PAN,
  LEFT_STAGE_TRANSIT,
};

enum class HomeReturnOrigin
{
  DIRECT,
  RECLAIM,
};

inline constexpr bool homeReturnUsesLeftStage(const HomeReturnOrigin origin)
{
  return origin == HomeReturnOrigin::RECLAIM;
}

inline constexpr bool homeReturnUsesInstrumentStage(
  const HomeReturnOrigin origin)
{
  return origin == HomeReturnOrigin::RECLAIM;
}

// The post-lift route is deliberately fixed. Keeping this list pure makes the
// safety-relevant order independently testable without MoveIt or robot hardware.
inline constexpr std::array<ReturnHomeExitPhase, 3> returnHomePostLiftPhases()
{
  return {
    ReturnHomeExitPhase::INSTRUMENT_STAGE,
    ReturnHomeExitPhase::LEFT_STAGE_PAN,
    ReturnHomeExitPhase::LEFT_STAGE_TRANSIT,
  };
}

// ReturnToolHome can determine the destination side before it touches the tool.
// Right-hand slots need the additional cached Left-Stage -> Home leg; left-hand
// slots deliberately finish their shared exit at Left-Stage.
inline bool returnHomeNeedsHomeTransit(
  const double release_world_x, const double right_side_world_x)
{
  return std::isfinite(release_world_x) &&
         std::isfinite(right_side_world_x) &&
         release_world_x > right_side_world_x;
}

enum class PostGraspFailureDisposition
{
  KEEP_HOLDING,
  CONFIRMED_NOT_HELD,
};

// A motion/planning failure is not evidence that the grasp failed. The robot
// may open automatically only after the gripper check confirms that no tool is
// held; otherwise it must remain closed for an explicit recovery action.
inline constexpr PostGraspFailureDisposition postGraspFailureDisposition(
  const std::optional<bool> tool_is_still_held)
{
  return tool_is_still_held.has_value() && !*tool_is_still_held ?
         PostGraspFailureDisposition::CONFIRMED_NOT_HELD :
         PostGraspFailureDisposition::KEEP_HOLDING;
}

inline bool isValidSixJointPose(const std::vector<double> & joints)
{
  if (joints.size() != 6) {
    return false;
  }
  for (const double value : joints) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

// Build a fixed transit target while retaining one rotation from the preceding
// planned state. For the Reclaim exit this is wrist_3: rotating it changes only
// the gripper/tool roll, not the taught collision-clear arm posture.
inline std::optional<std::vector<double>> jointTargetPreserving(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & fixed_target,
  const std::vector<double> & previous_state,
  const std::string & preserved_joint)
{
  if (joint_names.size() != fixed_target.size() ||
    joint_names.size() != previous_state.size())
  {
    return std::nullopt;
  }
  const auto it = std::find(
    joint_names.begin(), joint_names.end(), preserved_joint);
  if (it == joint_names.end()) {
    return std::nullopt;
  }
  const auto index = static_cast<std::size_t>(
    std::distance(joint_names.begin(), it));
  std::vector<double> target = fixed_target;
  target[index] = previous_state[index];
  return target;
}

}  // namespace execution
}  // namespace tracking_pkg
