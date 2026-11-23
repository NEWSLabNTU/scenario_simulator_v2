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

#ifndef TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_PEDESTRIAN_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_PEDESTRIAN_ENTITY_HPP_

#include <string>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
/**
 * A pedestrian the simulator side moves, not traffic_simulator: controller "model" (a behaviour
 * model run beside the simulator on its truth, whose commands the simulator integrates; scensim
 * 092 K12). traffic_simulator does not move it: it runs no behavior plugin (the controller's
 * name is not one), and adopts the pose and twist the simulator returns in UpdateEntityStatus,
 * as for every non-ego entity. The simulator learns the controller from the spawn's behavior.
 * The model decides where the pedestrian goes and how fast, so routes, speed changes, walking
 * straight and trajectories requested by the scenario are semantic errors, not ignored.
 */
class SimulatorDrivenPedestrianEntity : public PedestrianEntity
{
public:
  static auto behaviorName() noexcept -> const std::string &
  {
    static const std::string name = "model";
    return name;
  }

  /// True for the controller names this entity type stands for.
  static auto drives(const std::string & behavior) noexcept -> bool
  {
    return behavior == behaviorName();
  }

  explicit SimulatorDrivenPedestrianEntity(
    const std::string & name, const CanonicalizedEntityStatus &,
    const traffic_simulator_msgs::msg::PedestrianParameters &,
    const std::string & behavior = behaviorName());

  ~SimulatorDrivenPedestrianEntity() override = default;

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

  void requestWalkStraight() override;

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
  [[noreturn]] auto refuse(const std::string & action) const -> void;

  const std::string behavior_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__SIMULATOR_DRIVEN_PEDESTRIAN_ENTITY_HPP_
