#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <limits>

#include "tracking_pkg/preflight_retry.hpp"
#include "tracking_pkg/return_home_route.hpp"

namespace
{

using tracking_pkg::execution::runPreflightAttempts;

TEST(ReclaimPreflightRetry, RetriesCompleteRoundAndCanSucceed) {
  const std::vector<std::string> candidates{"awl"};
  std::vector<std::string> calls;
  int failed_rounds = 0;

  const auto result = runPreflightAttempts(
    candidates, 2,
    [&](const std::string & candidate, int attempt, std::string & reason) {
      calls.push_back(candidate + ":" + std::to_string(attempt));
      if (attempt == 2) {return true;}
      reason = candidate + " failed";
      return false;
    },
    [&](int, int, const std::vector<std::string> &, bool will_retry) {
      ++failed_rounds;
      EXPECT_TRUE(will_retry);
    });

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.attempts_run, 2);
  EXPECT_EQ(calls, (std::vector<std::string>{"awl:1", "awl:2"}));
  EXPECT_EQ(failed_rounds, 1);
}

TEST(ReclaimPreflightRetry, RetriesAllCandidatesInConfidenceOrder) {
  const std::vector<std::string> candidates{"first", "second"};
  std::vector<std::string> calls;
  std::vector<bool> retry_flags;

  const auto result = runPreflightAttempts(
    candidates, 2,
    [&](const std::string & candidate, int attempt, std::string & reason) {
      calls.push_back(candidate + ":" + std::to_string(attempt));
      reason = candidate + " failed";
      return false;
    },
    [&](int, int, const std::vector<std::string> & rejections,
    bool will_retry) {
      EXPECT_EQ(rejections.size(), 2u);
      retry_flags.push_back(will_retry);
    });

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.attempts_run, 2);
  EXPECT_EQ(
    calls, (std::vector<std::string>{
      "first:1", "second:1", "first:2", "second:2"}));
  EXPECT_EQ(retry_flags, (std::vector<bool>{true, false}));
  ASSERT_EQ(result.rejection_logs.size(), 2u);
  EXPECT_EQ(result.rejection_logs[0].size(), 2u);
  EXPECT_EQ(result.rejection_logs[1].size(), 2u);
}

TEST(ReclaimPreflightRetry, SingleAttemptDoesNotRetry) {
  const std::vector<std::string> candidates{"instrument"};
  int planner_calls = 0;
  int observer_calls = 0;

  const auto result = runPreflightAttempts(
    candidates, 1,
    [&](const std::string &, int, std::string & reason) {
      ++planner_calls;
      reason = "failed";
      return false;
    },
    [&](int, int, const std::vector<std::string> &, bool will_retry) {
      ++observer_calls;
      EXPECT_FALSE(will_retry);
    });

  EXPECT_FALSE(result.success);
  EXPECT_EQ(planner_calls, 1);
  EXPECT_EQ(observer_calls, 1);
}

TEST(ReclaimPreflightRetry, RejectsInvalidAttemptCount) {
  const std::vector<std::string> candidates{"awl"};
  EXPECT_THROW(
    runPreflightAttempts(
      candidates, 0,
      [](const std::string &, int, std::string &) {return false;},
      [](int, int, const std::vector<std::string> &, bool) {}),
    std::invalid_argument);
}

TEST(ReturnHomeRoute, UsesFixedPostLiftPhaseOrder) {
  using tracking_pkg::execution::ReturnHomeExitPhase;
  const auto phases = tracking_pkg::execution::returnHomePostLiftPhases();
  ASSERT_EQ(phases.size(), 3u);
  EXPECT_EQ(phases[0], ReturnHomeExitPhase::INSTRUMENT_STAGE);
  EXPECT_EQ(phases[1], ReturnHomeExitPhase::LEFT_STAGE_PAN);
  EXPECT_EQ(phases[2], ReturnHomeExitPhase::LEFT_STAGE_TRANSIT);
}

TEST(ReturnHomeRoute, ValidatesSixFiniteStageJoints) {
  using tracking_pkg::execution::isValidSixJointPose;
  EXPECT_TRUE(isValidSixJointPose({
    4.8766698837, -1.1527752441, 1.1332219283,
    -1.5510326673, -1.5708482901, -2.9439778964}));
  EXPECT_FALSE(isValidSixJointPose({1.0, 2.0, 3.0, 4.0, 5.0}));
  EXPECT_FALSE(isValidSixJointPose({
    1.0, 2.0, 3.0, 4.0, 5.0,
    std::numeric_limits<double>::quiet_NaN()}));
}

}  // namespace
