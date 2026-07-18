#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace tracking_pkg
{
namespace execution
{

constexpr double kUrElbowUpperLimitRad = 3.14159265358979323846;

inline bool isValidInstrumentElbowMinimum(double minimum_rad)
{
  return std::isfinite(minimum_rad) && minimum_rad >= 0.0 &&
         minimum_rad < kUrElbowUpperLimitRad;
}

struct JointMinimumCheck
{
  bool valid{false};
  double minimum{std::numeric_limits<double>::quiet_NaN()};
  double final {std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
};

// Validate every point, not only the IK goal. This is the final safety net
// behind MoveIt's path constraint: a planner or Cartesian interpolation must
// never smuggle an elbow-down sample into an accepted instrument pick.
inline JointMinimumCheck checkJointMinimum(
  const std::vector<std::string> & joint_names,
  const std::vector<std::vector<double>> & trajectory_points,
  const std::string & joint_name,
  double required_minimum,
  double numerical_tolerance = 1e-9)
{
  JointMinimumCheck result;
  if (!isValidInstrumentElbowMinimum(required_minimum)) {
    result.reason = "invalid required joint minimum";
    return result;
  }
  const auto joint_it = std::find(joint_names.begin(), joint_names.end(), joint_name);
  if (joint_it == joint_names.end()) {
    result.reason = "trajectory is missing joint '" + joint_name + "'";
    return result;
  }
  if (trajectory_points.empty()) {
    result.reason = "trajectory has no points";
    return result;
  }

  const std::size_t joint_index = static_cast<std::size_t>(
    std::distance(joint_names.begin(), joint_it));
  result.minimum = std::numeric_limits<double>::infinity();
  for (const auto & positions : trajectory_points) {
    if (positions.size() != joint_names.size()) {
      result.reason = "trajectory joint name/value size mismatch";
      return result;
    }
    const double value = positions[joint_index];
    if (!std::isfinite(value)) {
      result.reason = "trajectory contains a non-finite joint value";
      return result;
    }
    result.minimum = std::min(result.minimum, value);
    result.final = value;
    if (value + numerical_tolerance < required_minimum) {
      result.reason = "joint drops below required minimum";
      return result;
    }
  }
  result.valid = true;
  return result;
}

}  // namespace execution
}  // namespace tracking_pkg
