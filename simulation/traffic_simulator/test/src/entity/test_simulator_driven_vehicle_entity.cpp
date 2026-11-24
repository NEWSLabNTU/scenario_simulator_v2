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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/simulator_driven_vehicle_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/pose.hpp>

#include "../catalogs.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

using traffic_simulator::entity::SimulatorDrivenVehicleEntity;

/// Vehicles a behaviour model commands (scensim 092 K13): controller "model".
class ModelVehicleTest : public testing::Test
{
protected:
  ModelVehicleTest()
  : name((activateLaneletWrapper("standard_map"), "follower")),
    pose(makeCanonicalizedLaneletPose(120659)),
    bbox(makeBoundingBox()),
    status(makeCanonicalizedEntityStatus(
      pose, bbox, 0.0, name, traffic_simulator_msgs::msg::EntityType::VEHICLE)),
    car(name, status, getVehicleParameters(), SimulatorDrivenVehicleEntity::modelBehaviorName()),
    autopilot(name, status, getVehicleParameters())
  {
  }

  const std::string name;
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  traffic_simulator::entity_status::CanonicalizedEntityStatus status;
  SimulatorDrivenVehicleEntity car;
  SimulatorDrivenVehicleEntity autopilot;
};

/// "model" is one of the controllers the simulator drives; the entity reports it.
TEST_F(ModelVehicleTest, controllerModelIsSimulatorDriven)
{
  EXPECT_TRUE(SimulatorDrivenVehicleEntity::drives("model"));
  EXPECT_TRUE(SimulatorDrivenVehicleEntity::drives("simulator_autopilot"));
  EXPECT_TRUE(SimulatorDrivenVehicleEntity::drives("agent"));
  EXPECT_FALSE(SimulatorDrivenVehicleEntity::drives(""));
  EXPECT_EQ(car.getCurrentAction(), "model");
  EXPECT_EQ(car.getEntityType().type, traffic_simulator_msgs::msg::EntityType::VEHICLE);
}

/// The status the simulator returned survives the frame's update: pose and twist unchanged.
TEST_F(ModelVehicleTest, adoptsTheReturnedStatus)
{
  auto returned = makeEntityStatus(
    traffic_simulator::pose::toMapPose(pose), bbox, 12.5, name,
    traffic_simulator_msgs::msg::EntityType::VEHICLE);
  returned.pose.position.x += 0.6;
  returned.action_status.accel.linear.x = -2.0;
  car.setStatus(returned);
  car.onUpdate(0.05, 0.05);
  EXPECT_DOUBLE_EQ(car.getCurrentTwist().linear.x, 12.5);
  EXPECT_DOUBLE_EQ(car.getCurrentAccel().linear.x, -2.0);
  EXPECT_DOUBLE_EQ(car.getMapPose().position.x, returned.pose.position.x);
}

/// The model decides the motion: goal, route and speed requests are semantic errors, and none
/// reaches the simulator as a goal update; the autopilot still takes them.
TEST_F(ModelVehicleTest, motionRequestsAreRefusedForAModelOnly)
{
  const traffic_simulator::RouteOption option;
  const auto map_pose = traffic_simulator::pose::toMapPose(pose);
  EXPECT_THROW(car.requestSpeedChange(10.0, false), common::SemanticError);
  EXPECT_THROW(
    car.requestSpeedChange(
      10.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      false),
    common::SemanticError);
  EXPECT_THROW(car.requestAcquirePosition(pose, option), common::SemanticError);
  EXPECT_THROW(car.requestAcquirePosition(map_pose, option), common::SemanticError);
  EXPECT_THROW(
    car.requestAssignRoute(std::vector<geometry_msgs::msg::Pose>{map_pose}, option),
    common::SemanticError);
  EXPECT_THROW(car.requestFollowTrajectory(nullptr), common::SemanticError);
  EXPECT_THROW(car.requestLaneChange(lanelet::Id()), common::SemanticError);
  EXPECT_NO_THROW(car.cancelRequest());
  EXPECT_TRUE(car.takeGoalUpdates().empty());

  EXPECT_NO_THROW(autopilot.requestSpeedChange(10.0, false));
  EXPECT_EQ(autopilot.takeGoalUpdates().size(), 1u);
}
