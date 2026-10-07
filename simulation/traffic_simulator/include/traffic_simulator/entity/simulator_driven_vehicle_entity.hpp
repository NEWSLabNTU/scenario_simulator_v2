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

#ifndef TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_VEHICLE_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_VEHICLE_ENTITY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <string>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
/**
 * A vehicle the simulator drives with its own driver (controller "simulator_autopilot"; for
 * CARLA, Traffic Manager). traffic_simulator does not move it: it runs no behavior, adopts
 * the pose the simulator returns in UpdateEntityStatus (as for every non-ego entity), and
 * turns the scenario's goal, route and absolute speed requests into GoalUpdates, which
 * API::updateFrame sends to the simulator as UpdateEntityGoal before the frame's
 * UpdateEntityStatus. Requests the simulator's driver cannot honor (lane changes, relative
 * speeds, trajectories) are semantic errors, not silently ignored.
 */
class SimulatorDrivenVehicleEntity : public VehicleEntity
{
public:
  struct GoalUpdate
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;  // empty: route unchanged (or cleared)
    bool clear = false;                               // drop the route
    std::optional<double> target_speed;               // absolute [m/s]
  };

  static auto behaviorName() noexcept -> const std::string &
  {
    static const std::string name = "simulator_autopilot";
    return name;
  }

  explicit SimulatorDrivenVehicleEntity(
    const std::string & name, const CanonicalizedEntityStatus &,
    const traffic_simulator_msgs::msg::VehicleParameters &);

  ~SimulatorDrivenVehicleEntity() override = default;

  /// The updates requested since the last call, oldest first.
  auto takeGoalUpdates() -> std::vector<GoalUpdate>;

  void cancelRequest() override;

  auto getCurrentAction() const -> std::string override;

  auto onUpdate(const double current_time, const double step_time) -> void override;

  void requestAcquirePosition(const CanonicalizedLaneletPose &, const RouteOption &) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose &, const RouteOption &) override;

  void requestAssignRoute(
    const std::vector<geometry_msgs::msg::Pose> &, const RouteOption &) override;

  void requestAssignRoute(
    const std::vector<CanonicalizedLaneletPose> &, const RouteOption &) override;

  auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void override;

  auto requestLaneChange(const lanelet::Id) -> void override;

  auto requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void override;

  auto requestSpeedChange(
    const double target_speed, const speed_change::Transition,
    const speed_change::Constraint, const bool continuous) -> void override;

  void requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
    const speed_change::Constraint, const bool continuous) override;

  void requestSpeedChange(const double target_speed, const bool continuous) override;

  void requestSpeedChange(
    const speed_change::RelativeTargetSpeed &, const bool continuous) override;

private:
  std::vector<GoalUpdate> goal_updates_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_VEHICLE_ENTITY_HPP_
