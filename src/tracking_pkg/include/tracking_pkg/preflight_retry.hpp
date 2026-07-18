#ifndef TRACKING_PKG__PREFLIGHT_RETRY_HPP_
#define TRACKING_PKG__PREFLIGHT_RETRY_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace tracking_pkg
{
namespace execution
{

struct PreflightRetryResult
{
  bool success{false};
  int attempts_run{0};
  std::size_t candidate_index{0};
  std::vector<std::vector<std::string>> rejection_logs;
};

// Run the complete candidate sequence again after a failed planning round.
// The helper deliberately knows nothing about MoveIt: callers decide what one
// candidate attempt means and can therefore rebuild every motion leg each time.
template<typename Candidate, typename Planner, typename AttemptObserver>
PreflightRetryResult runPreflightAttempts(
  const std::vector<Candidate> & candidates,
  int total_attempts,
  Planner planner,
  AttemptObserver observe_failed_attempt)
{
  if (total_attempts < 1) {
    throw std::invalid_argument("total_attempts must be at least 1");
  }

  PreflightRetryResult result;
  for (int attempt = 1; attempt <= total_attempts; ++attempt) {
    result.attempts_run = attempt;
    result.rejection_logs.emplace_back();
    auto & rejections = result.rejection_logs.back();

    for (std::size_t candidate_index = 0;
      candidate_index < candidates.size(); ++candidate_index)
    {
      std::string rejection;
      if (planner(candidates[candidate_index], attempt, rejection)) {
        result.success = true;
        result.candidate_index = candidate_index;
        return result;
      }
      rejections.push_back(rejection);
    }

    observe_failed_attempt(
      attempt, total_attempts, rejections, attempt < total_attempts);
  }
  return result;
}

}  // namespace execution
}  // namespace tracking_pkg

#endif  // TRACKING_PKG__PREFLIGHT_RETRY_HPP_
