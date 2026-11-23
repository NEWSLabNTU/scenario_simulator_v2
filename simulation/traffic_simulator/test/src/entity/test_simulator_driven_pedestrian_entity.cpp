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
#include <traffic_simulator/entity/simulator_driven_pedestrian_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/pose.hpp>

#include "../catalogs.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

using traffic_simulator::entity::SimulatorDrivenPedestrianEntity;

class SimulatorDrivenPedestrianEntityTest : public testing::Test
{
protected:
  SimulatorDrivenPedestrianEntityTest()
  : name((activateLaneletWrapper("standard_map"), "walker")),
    pose(makeCanonicalizedLaneletPose(120659)),
    bbox(makeBoundingBox()),
    status(makeCanonicalizedEntityStatus(
      pose, bbox, 0.0, name, traffic_simulator_msgs::msg::EntityType::PEDESTRIAN)),
    walker(name, status, getPedestrianParameters())
  {
  }

  const std::string name;
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  traffic_simulator::entity_status::CanonicalizedEntityStatus status;
  SimulatorDrivenPedestrianEntity walker;
};

/// The controller name is not a plugin: the entity is built on do-nothing, and reports it.
TEST_F(SimulatorDrivenPedestrianEntityTest, controllerModelLoadsNoPlugin)
{
  EXPECT_TRUE(SimulatorDrivenPedestrianEntity::drives("model"));
  EXPECT_FALSE(SimulatorDrivenPedestrianEntity::drives(""));
  EXPECT_FALSE(SimulatorDrivenPedestrianEntity::drives("simulator_autopilot"));
  EXPECT_EQ(walker.getCurrentAction(), "model");
  EXPECT_EQ(walker.getEntityType().type, traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
}

/// The status the simulator returned survives the frame's update: pose and twist unchanged.
TEST_F(SimulatorDrivenPedestrianEntityTest, adoptsTheReturnedStatus)
{
  auto returned = makeEntityStatus(
    traffic_simulator::pose::toMapPose(pose), bbox, 1.3, name,
    traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
  returned.pose.position.x += 0.25;
  returned.action_status.accel.linear.x = 0.4;
  walker.setStatus(returned);
  walker.onUpdate(0.05, 0.05);
  EXPECT_DOUBLE_EQ(walker.getCurrentTwist().linear.x, 1.3);
  EXPECT_DOUBLE_EQ(walker.getCurrentAccel().linear.x, 0.4);
  EXPECT_DOUBLE_EQ(walker.getMapPose().position.x, returned.pose.position.x);
  EXPECT_DOUBLE_EQ(walker.getMapPose().position.y, returned.pose.position.y);
}

/// The model decides the motion: the scenario's motion requests are semantic errors.
TEST_F(SimulatorDrivenPedestrianEntityTest, motionRequestsAreRefused)
{
  const traffic_simulator::RouteOption option;
  EXPECT_THROW(walker.requestSpeedChange(1.0, false), common::SemanticError);
  EXPECT_THROW(
    walker.requestSpeedChange(
      1.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      false),
    common::SemanticError);
  EXPECT_THROW(walker.requestWalkStraight(), common::SemanticError);
  EXPECT_THROW(walker.requestAcquirePosition(pose, option), common::SemanticError);
  EXPECT_THROW(
    walker.requestAssignRoute(std::vector<geometry_msgs::msg::Pose>{}, option),
    common::SemanticError);
  EXPECT_THROW(walker.requestFollowTrajectory(nullptr), common::SemanticError);
  EXPECT_NO_THROW(walker.cancelRequest());
}
