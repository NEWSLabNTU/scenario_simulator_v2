// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <system_error>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

namespace traffic_simulator
{
namespace
{
/*
   Where a clock that follows simulation time leaves the last time it published, so the
   next one (usually the next scenario's interpreter process) starts no earlier. Keyed by
   ROS domain because /clock, and whoever consumes it, is per domain.
*/
auto lastPublishedTimePath() -> std::filesystem::path
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  std::error_code error;
  auto directory = std::filesystem::temp_directory_path(error);
  if (error) {
    directory = "/tmp";
  }
  return directory /
         ("scenario_simulator_v2_last_clock_domain" + std::string(domain ? domain : "0"));
}
}  // namespace

SimulationClock::SimulationClock(
  bool use_sim_time, double realtime_factor, double frame_rate, bool follows_simulation_time)
: rclcpp::Clock(RCL_ROS_TIME),
  use_sim_time(use_sim_time),
  follows_simulation_time(not use_sim_time and follows_simulation_time),
  realtime_factor(realtime_factor),
  frame_rate_(frame_rate),
  time_at_the_start_of_the_simulator_(makeStartTime())
{
}

auto SimulationClock::makeStartTime() -> rclcpp::Time
{
  if (use_sim_time) {
    return rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME);
  } else if (not follows_simulation_time) {
    return rclcpp::Time(now().nanoseconds(), RCL_ROS_TIME);
  } else {
    /*
       Continue from the last time an earlier clock of this kind published in this
       domain, one step later, rather than from wall time. A long-lived consumer (the
       ego's Autoware, reused across scenarios) then sees the gap between two scenarios
       as a pause, which is what it was: nothing in the simulated world moved.

       Starting from max(wall now, last published) instead made /clock leap forward by
       the whole idle gap -- ~30 s between consecutive scenarios, hours after a break --
       and every stamp Autoware held became that old at once. Measured on the first
       tick of each scenario: all eleven topic_state_monitors timed out (timeout 30 s),
       multi_object_tracker reported a detection delay equal to the gap, the EKF flagged
       its pose and twist inputs as delayed with a large covariance ellipse, and the
       planning and control validators rejected a trajectory that old -- about twenty
       ERROR onsets and an EMERGENCY_STOP per scenario, before the ego even existed.

       Wall time is only the fallback for the first clock in a domain.
    */
    auto start = now().nanoseconds();
    if (std::ifstream file(lastPublishedTimePath()); file) {
      std::int64_t last_published = 0;
      if (file >> last_published and last_published > 0) {
        const auto step = static_cast<std::int64_t>(getStepTime() * 1e9);
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("simulation_clock"),
          "/clock continues from " << last_published / 1e9
                                   << ", where an earlier scenario in this domain stopped ("
                                   << (start - last_published) / 1e9
                                   << " s behind wall time); the time between scenarios is a "
                                      "pause, not simulated time.");
        start = last_published + step;
      }
    }
    return rclcpp::Time(start, RCL_ROS_TIME);
  }
}

auto SimulationClock::persistCurrentRosTime() -> void
{
  // Best effort: losing this only weakens the cross-scenario monotonicity guarantee.
  if (std::ofstream file(lastPublishedTimePath(), std::ios::trunc); file) {
    file << getCurrentRosTime().nanoseconds() << std::endl;
  }
}

auto SimulationClock::update() -> void
{
  seconds_since_the_simulator_started_ += realtime_factor / frame_rate_;
  if (follows_simulation_time) {
    persistCurrentRosTime();
  }
}

auto SimulationClock::getCurrentRosTimeAsMsg() -> rosgraph_msgs::msg::Clock
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = getCurrentRosTime();
  return clock;
}

auto SimulationClock::getCurrentRosTime() -> rclcpp::Time
{
  if (not use_sim_time and not follows_simulation_time) {
    return now();
  } else {
    return time_at_the_start_of_the_simulator_ +
           rclcpp::Duration::from_seconds(getCurrentSimulationTime());
  }
}

auto SimulationClock::start() -> void
{
  if (started()) {
    THROW_SIMULATION_ERROR(
      "npc logic is already started. Please check simulation clock instance was destroyed.");
  } else {
    seconds_at_the_start_of_the_scenario_ = seconds_since_the_simulator_started_;
  }
}
}  // namespace traffic_simulator
