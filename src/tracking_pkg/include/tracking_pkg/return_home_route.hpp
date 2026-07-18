#pragma once

#include <array>
#include <cmath>
#include <optional>
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

}  // namespace execution
}  // namespace tracking_pkg
