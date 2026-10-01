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

#ifndef TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
#define TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_

#include <cstdint>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

namespace traffic_simulator
{
/*
   Where ROS time comes from (NEWSLabNTU fork). Only getCurrentRosTime() -- and so the
   /clock this simulator publishes, signal stamps and entity stamps -- depends on it;
   scenario time (getCurrentScenarioTime, getCurrentSimulationTime) always counts frames
   times step time, whatever the source.

   frames                   Stock behaviour: wall time while use_sim_time is false,
                            frames times step time from 0 while it is true.
   follows_simulation_time  Phase 014: wall time at construction (or one step after the
                            last value an earlier clock persisted under $TMP) plus frames
                            times step time.
   simulator                Phase 015: the last non-zero simulation_time the simulator
                            reported in InitializeResponse / UpdateFrameResponse (CARLA's
                            elapsed_seconds plus csb's episode epoch). Nothing is persisted
                            and API publishes no /clock: the simulator side (acb) owns it.
                            Wall time until the first value arrives, said once.
*/
enum class ClockSource { frames, follows_simulation_time, simulator };

/*
   The value of the clock_source parameter, with the 014 parameter folded in:
   clock_follows_simulation_time:=true with clock_source left at "frames" means
   follows_simulation_time. Throws on an unknown name, and on clock_source:=simulator
   together with clock_follows_simulation_time:=true (the two are mutually exclusive).
*/
auto toClockSource(const std::string & clock_source, bool clock_follows_simulation_time)
  -> ClockSource;

class SimulationClock : rclcpp::Clock
{
public:
  /*
     follows_simulation_time (NEWSLabNTU fork) only matters while use_sim_time is
     false. It makes getCurrentRosTime(), and so the /clock this simulator publishes,
     advance with simulated time -- the wall time at construction plus frames times
     step time -- instead of reading the wall clock. A consumer running on /clock then
     sees time pass at the rate the simulated world moves, even when the frame loop
     falls behind real time.

     The start time continues from the last time published by any earlier clock of
     this kind in the same ROS domain (kept in a small file under the temp
     directory), one step later, so a long-lived consumer sees consecutive scenarios
     as one continuous timeline: never backwards, and never a forward leap over the
     idle time between runs. Wall time is used only when no earlier clock exists.
  */
  explicit SimulationClock(
    bool use_sim_time, double realtime_factor, double frame_rate,
    ClockSource clock_source = ClockSource::frames);

  // 014 signature, kept for compatibility: true means ClockSource::follows_simulation_time.
  explicit SimulationClock(
    bool use_sim_time, double realtime_factor, double frame_rate, bool follows_simulation_time);

  /*
     Record the simulation_time of a simulator response (seconds). 0 means the
     simulator does not know (a failed tick, or a backend without the field): the last
     value is kept rather than going back. Ignored unless clock_source is simulator.
  */
  auto setSimulatorTime(double seconds) -> void;

  auto hasSimulatorTime() const { return simulator_time_nanoseconds_ > 0; }

  auto getCurrentRosTime() -> rclcpp::Time;

  auto getCurrentRosTimeAsMsg() -> rosgraph_msgs::msg::Clock;

  auto getCurrentScenarioTime() const
  {
    return seconds_since_the_simulator_started_ - seconds_at_the_start_of_the_scenario_;
  }

  auto getCurrentSimulationTime() const { return seconds_since_the_simulator_started_; }

  auto getStepTime() const { return realtime_factor / frame_rate_; }

  auto start() -> void;

  auto started() const { return not std::isnan(seconds_at_the_start_of_the_scenario_); }

  auto update() -> void;

  const bool use_sim_time;

  const ClockSource clock_source;

  const bool follows_simulation_time;

  double realtime_factor;

private:
  auto makeStartTime() -> rclcpp::Time;

  auto persistCurrentRosTime() -> void;

  double frame_rate_;

  const rclcpp::Time time_at_the_start_of_the_simulator_;

  double seconds_since_the_simulator_started_ = 0.0;

  double seconds_at_the_start_of_the_scenario_ = std::numeric_limits<double>::quiet_NaN();

  std::int64_t simulator_time_nanoseconds_ = 0;

  bool said_simulator_time_missing_ = false;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
