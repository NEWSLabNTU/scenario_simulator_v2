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

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

/**
 * @note Test basic functionality used in API.
 * Test initialization logic by calling update without initialized clock
 * - the goal is to verify that mandatory initialization works.
 */
TEST(SimulationClock, SimulationClock)
{
  auto simulation_clock = traffic_simulator::SimulationClock(true, 1.0, 10.0);

  EXPECT_FALSE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_NO_THROW(simulation_clock.start());

  EXPECT_TRUE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_THROW(simulation_clock.start(), common::SimulationError);
}

/**
 * @note Test basic functionality used in API. Test time obtaining correctness with initialized object
 * and not using raw clock - the goal is to test whether the time has increased
 * according to the times of update function calls.
 */
TEST(SimulationClock, getCurrentRosTime)
{
  const double realtime_factor = 2.0;
  const double frame_rate = 10.0;
  auto simulation_clock = traffic_simulator::SimulationClock(true, realtime_factor, frame_rate);
  simulation_clock.start();

  const auto initial_time = simulation_clock.getCurrentRosTime();

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  EXPECT_NEAR(
    (simulation_clock.getCurrentRosTime() - initial_time).seconds(),
    static_cast<double>(iterations) * realtime_factor / frame_rate, 1e-6);
}

/**
 * @note Test basic functionality used in API. Test scenario time calculation correctness with initialized object,
 * npc logic started after several update() calls and additional update() calls after starting npc logic.
 */
TEST(SimulationClock, getCurrentScenarioTime)
{
  const double realtime_factor = 1.0;
  const double frame_rate = 30.0;
  auto simulation_clock = traffic_simulator::SimulationClock(true, realtime_factor, frame_rate);

  simulation_clock.start();

  EXPECT_DOUBLE_EQ(simulation_clock.getCurrentScenarioTime(), 0.0);

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  EXPECT_NEAR(
    static_cast<double>(iterations) * realtime_factor / frame_rate,
    simulation_clock.getCurrentScenarioTime(), 1e-6);
}

/**
 * @note Test basic functionality used in API. Test updating correctness with initialized object
 * by calling update several times expecting the current time to increase accordingly.
 */
TEST(SimulationClock, Update)
{
  auto simulation_clock = traffic_simulator::SimulationClock(true, 1.0, 10.0);

  simulation_clock.start();

  const double initial_simulation_time = simulation_clock.getCurrentSimulationTime();
  const double step_time = simulation_clock.getStepTime();

  for (int i = 0; i < 10; ++i) {
    simulation_clock.update();
    const double expected_simulation_time =
      initial_simulation_time + static_cast<double>(i + 1) * step_time;
    const double actual_simulation_time = simulation_clock.getCurrentSimulationTime();
    EXPECT_NEAR(actual_simulation_time, expected_simulation_time, 1e-6);
  }
}

/**
 * @note NEWSLabNTU fork: the clock_source parameter. "frames" keeps the stock behaviour,
 * clock_follows_simulation_time:=true still selects follows_simulation_time, and the two
 * simulator-time settings are mutually exclusive.
 */
TEST(SimulationClock, toClockSource)
{
  using traffic_simulator::ClockSource;
  using traffic_simulator::toClockSource;
  EXPECT_EQ(toClockSource("frames", false), ClockSource::frames);
  EXPECT_EQ(toClockSource("frames", true), ClockSource::follows_simulation_time);
  EXPECT_EQ(
    toClockSource("follows_simulation_time", false), ClockSource::follows_simulation_time);
  EXPECT_EQ(toClockSource("simulator", false), ClockSource::simulator);
  EXPECT_THROW(toClockSource("simulator", true), common::SimulationError);
  EXPECT_THROW(toClockSource("carla", false), common::SimulationError);
}

/**
 * @note clock_source frames with use_sim_time false: ROS time is wall time, frames do not
 * move it, and reported simulator times are ignored.
 */
TEST(SimulationClock, framesSourceIsWallTime)
{
  auto simulation_clock =
    traffic_simulator::SimulationClock(false, 1.0, 10.0, traffic_simulator::ClockSource::frames);
  simulation_clock.start();
  simulation_clock.setSimulatorTime(5'000'000'000);
  EXPECT_FALSE(simulation_clock.hasSimulatorTime());

  const auto wall_before = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  for (int i = 0; i < 100; ++i) {
    simulation_clock.update();
  }
  const auto ros_time = simulation_clock.getCurrentRosTime().nanoseconds();
  const auto wall_after = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  // 100 frames are 10 simulated seconds; wall time moved far less than that.
  EXPECT_GE(ros_time, wall_before);
  EXPECT_LE(ros_time, wall_after);
}

/**
 * @note clock_source follows_simulation_time: ROS time advances one step per frame from
 * its start, independent of wall time.
 */
TEST(SimulationClock, followsSimulationTimeSourceCountsFrames)
{
  auto simulation_clock = traffic_simulator::SimulationClock(
    false, 1.0, 20.0, traffic_simulator::ClockSource::follows_simulation_time);
  EXPECT_TRUE(simulation_clock.follows_simulation_time);
  simulation_clock.start();
  const auto initial_time = simulation_clock.getCurrentRosTime();
  for (int i = 0; i < 40; ++i) {
    simulation_clock.update();
  }
  EXPECT_NEAR((simulation_clock.getCurrentRosTime() - initial_time).seconds(), 2.0, 1e-6);
}

/**
 * @note The 014 constructor still selects follows_simulation_time.
 */
TEST(SimulationClock, boolConstructorKeepsFollowsSimulationTime)
{
  auto follows = traffic_simulator::SimulationClock(false, 1.0, 10.0, true);
  EXPECT_EQ(follows.clock_source, traffic_simulator::ClockSource::follows_simulation_time);
  auto frames = traffic_simulator::SimulationClock(false, 1.0, 10.0, false);
  EXPECT_EQ(frames.clock_source, traffic_simulator::ClockSource::frames);
}

/**
 * @note clock_source simulator: wall time until the simulator reports a time, then exactly
 * the last non-zero simulation_time; 0 (unknown) keeps the last value; frames do not move ROS
 * time, but scenario time still counts frames.
 */
TEST(SimulationClock, simulatorSource)
{
  auto simulation_clock = traffic_simulator::SimulationClock(
    false, 1.0, 20.0, traffic_simulator::ClockSource::simulator);
  EXPECT_FALSE(simulation_clock.follows_simulation_time);

  // Before the first value: wall time.
  EXPECT_FALSE(simulation_clock.hasSimulatorTime());
  const auto wall_before = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  const auto fallback = simulation_clock.getCurrentRosTime().nanoseconds();
  const auto wall_after = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  EXPECT_GE(fallback, wall_before);
  EXPECT_LE(fallback, wall_after);

  // The first reported time (Initialize).
  simulation_clock.setSimulatorTime(1'234'500'000'000);
  EXPECT_TRUE(simulation_clock.hasSimulatorTime());
  EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), 1234500000000);
  EXPECT_EQ(simulation_clock.getCurrentRosTime().get_clock_type(), RCL_ROS_TIME);

  simulation_clock.start();
  for (int i = 0; i < 10; ++i) {
    simulation_clock.update();
  }
  // Frames alone do not move ROS time...
  EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), 1234500000000);
  // ...but they still are the scenario's time.
  EXPECT_NEAR(simulation_clock.getCurrentScenarioTime(), 0.5, 1e-9);

  // An exact frame time (UpdateFrame).
  simulation_clock.setSimulatorTime(1'234'550'000'000);
  EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), 1234550000000);

  // 0 means unknown: keep the last value rather than go back.
  simulation_clock.setSimulatorTime(0);
  EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), 1234550000000);
  EXPECT_EQ(simulation_clock.getCurrentRosTimeAsMsg().clock.sec, 1234);
  EXPECT_EQ(simulation_clock.getCurrentRosTimeAsMsg().clock.nanosec, 550000000u);

  // Off-grid values, as CARLA's f32 step makes them, are kept to the nanosecond: the
  // value is the simulator's integer, not a double converted here.
  for (const std::int64_t ns : {std::int64_t{171'600'002'557}, std::int64_t{237'171'600'002'557},
                                std::int64_t{1'790'611'998'049'999'872}}) {
    simulation_clock.setSimulatorTime(ns);
    EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), ns);
    const auto msg = simulation_clock.getCurrentRosTimeAsMsg();
    EXPECT_EQ(std::int64_t{msg.clock.sec} * 1'000'000'000 + msg.clock.nanosec, ns);
  }
  // Negative is "unknown" too.
  simulation_clock.setSimulatorTime(-1);
  EXPECT_EQ(simulation_clock.getCurrentRosTime().nanoseconds(), 1'790'611'998'049'999'872);
}

/**
 * @note clock_source simulator neither reads nor writes the file follows_simulation_time
 * persists under the temp directory.
 */
TEST(SimulationClock, simulatorSourceDoesNotPersist)
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const auto path = std::filesystem::temp_directory_path() /
                    ("scenario_simulator_v2_last_clock_domain" +
                     std::string(domain ? domain : "0"));
  std::error_code error;
  const bool existed = std::filesystem::exists(path, error);
  const auto before =
    existed ? std::filesystem::last_write_time(path, error) : std::filesystem::file_time_type{};

  auto simulation_clock = traffic_simulator::SimulationClock(
    false, 1.0, 20.0, traffic_simulator::ClockSource::simulator);
  simulation_clock.setSimulatorTime(10'000'000'000);
  simulation_clock.start();
  simulation_clock.update();

  EXPECT_EQ(std::filesystem::exists(path, error), existed);
  if (existed) {
    EXPECT_EQ(std::filesystem::last_write_time(path, error), before);
  }
}
