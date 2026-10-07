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

#include <iomanip>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/simulator_driven_vehicle_entity.hpp>
#include <utility>

namespace traffic_simulator
{
namespace entity
{
SimulatorDrivenVehicleEntity::SimulatorDrivenVehicleEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
: VehicleEntity(name, entity_status, parameters, BuiltinBehavior::doNothing())
{
}

auto SimulatorDrivenVehicleEntity::takeGoalUpdates() -> std::vector<GoalUpdate>
{
  return std::exchange(goal_updates_, {});
}

void SimulatorDrivenVehicleEntity::cancelRequest()
{
  GoalUpdate update;
  update.clear = true;
  goal_updates_.push_back(update);
}

auto SimulatorDrivenVehicleEntity::getCurrentAction() const -> std::string
{
  return behaviorName();
}

auto SimulatorDrivenVehicleEntity::onUpdate(const double current_time, const double step_time)
  -> void
{
  /*
     No behavior runs: the status is the one the simulator returned for this frame. The
     do-nothing plugin is not updated either, because it would zero the twist the simulator
     reported, and SpeedCondition reads that twist.
  */
  EntityBase::onUpdate(current_time, step_time);
  status_->setTime(current_time);
  EntityBase::onPostUpdate(current_time, step_time);
}

void SimulatorDrivenVehicleEntity::requestAcquirePosition(
  const CanonicalizedLaneletPose & lanelet_pose, const RouteOption & option)
{
  requestAcquirePosition(static_cast<geometry_msgs::msg::Pose>(lanelet_pose), option);
}

void SimulatorDrivenVehicleEntity::requestAcquirePosition(
  const geometry_msgs::msg::Pose & map_pose, const RouteOption &)
{
  GoalUpdate update;
  update.waypoints.push_back(map_pose);
  goal_updates_.push_back(update);
}

void SimulatorDrivenVehicleEntity::requestAssignRoute(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, const RouteOption &)
{
  if (waypoints.empty()) {
    THROW_SEMANTIC_ERROR("AssignRouteAction for ", std::quoted(name), " has no waypoints.");
  }
  GoalUpdate update;
  update.waypoints = waypoints;
  goal_updates_.push_back(update);
}

void SimulatorDrivenVehicleEntity::requestAssignRoute(
  const std::vector<CanonicalizedLaneletPose> & waypoints, const RouteOption & option)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const auto & waypoint : waypoints) {
    poses.push_back(static_cast<geometry_msgs::msg::Pose>(waypoint));
  }
  requestAssignRoute(poses, option);
}

auto SimulatorDrivenVehicleEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void
{
  THROW_SEMANTIC_ERROR(
    "FollowTrajectoryAction was requested for ", std::quoted(name),
    ", which is driven by the simulator (controller simulator_autopilot). Use the default "
    "controller for scripted trajectories.");
}

auto SimulatorDrivenVehicleEntity::requestLaneChange(const lanelet::Id) -> void
{
  THROW_SEMANTIC_ERROR(
    "LaneChangeAction was requested for ", std::quoted(name),
    ", which is driven by the simulator (controller simulator_autopilot); its driver decides "
    "lane changes. Use the default controller for scripted lane changes.");
}

auto SimulatorDrivenVehicleEntity::requestLaneChange(const lane_change::Parameter &) -> void
{
  requestLaneChange(lanelet::Id());
}

auto SimulatorDrivenVehicleEntity::requestSpeedChange(
  const double target_speed, const speed_change::Transition, const speed_change::Constraint,
  const bool continuous) -> void
{
  requestSpeedChange(target_speed, continuous);
}

void SimulatorDrivenVehicleEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & target_speed, const speed_change::Transition,
  const speed_change::Constraint, const bool continuous)
{
  requestSpeedChange(target_speed, continuous);
}

void SimulatorDrivenVehicleEntity::requestSpeedChange(const double target_speed, const bool)
{
  /// @note The simulator's driver chooses the transition; only the target is forwarded.
  GoalUpdate update;
  update.target_speed = target_speed;
  goal_updates_.push_back(update);
}

void SimulatorDrivenVehicleEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const bool)
{
  THROW_SEMANTIC_ERROR(
    "A relative SpeedAction was requested for ", std::quoted(name),
    ", which is driven by the simulator (controller simulator_autopilot). Only an absolute "
    "target speed is supported.");
}
}  // namespace entity
}  // namespace traffic_simulator
