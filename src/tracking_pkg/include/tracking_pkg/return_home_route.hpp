#pragma once

#include <array>
#include <cmath>
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
