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
#include <traffic_simulator/entity/simulator_driven_pedestrian_entity.hpp>

namespace traffic_simulator
{
namespace entity
{
SimulatorDrivenPedestrianEntity::SimulatorDrivenPedestrianEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & behavior)
: PedestrianEntity(name, entity_status, parameters, BuiltinBehavior::doNothing()),
  behavior_(behavior)
{
}

void SimulatorDrivenPedestrianEntity::cancelRequest() {}

auto SimulatorDrivenPedestrianEntity::getCurrentAction() const -> std::string { return behavior_; }

auto SimulatorDrivenPedestrianEntity::onUpdate(const double current_time, const double step_time)
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

auto SimulatorDrivenPedestrianEntity::refuse(const std::string & action) const -> void
{
  THROW_SEMANTIC_ERROR(
    action, " was requested for ", std::quoted(name), ", which is moved by its controller ",
    behavior_, ": its behaviour model decides where it goes and how fast.");
}

void SimulatorDrivenPedestrianEntity::requestAcquirePosition(
  const CanonicalizedLaneletPose &, const RouteOption &)
{
  refuse("AcquirePositionAction");
}

void SimulatorDrivenPedestrianEntity::requestAcquirePosition(
  const geometry_msgs::msg::Pose &, const RouteOption &)
{
  refuse("AcquirePositionAction");
}

void SimulatorDrivenPedestrianEntity::requestAssignRoute(
  const std::vector<geometry_msgs::msg::Pose> &, const RouteOption &)
{
  refuse("AssignRouteAction");
}

void SimulatorDrivenPedestrianEntity::requestAssignRoute(
  const std::vector<CanonicalizedLaneletPose> &, const RouteOption &)
{
  refuse("AssignRouteAction");
}

auto SimulatorDrivenPedestrianEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void
{
  refuse("FollowTrajectoryAction");
}

void SimulatorDrivenPedestrianEntity::requestWalkStraight() { refuse("WalkStraightAction"); }

auto SimulatorDrivenPedestrianEntity::requestSpeedChange(
  const double, const speed_change::Transition, const speed_change::Constraint, const bool) -> void
{
  refuse("SpeedAction");
}

void SimulatorDrivenPedestrianEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
  const speed_change::Constraint, const bool)
{
  refuse("SpeedAction");
}

void SimulatorDrivenPedestrianEntity::requestSpeedChange(const double, const bool)
{
  refuse("SpeedAction");
}

void SimulatorDrivenPedestrianEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const bool)
{
  refuse("SpeedAction");
}
}  // namespace entity
}  // namespace traffic_simulator
