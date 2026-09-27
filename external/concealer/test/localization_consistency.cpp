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

#include <cmath>
#include <concealer/field_operator_application.hpp>

static auto pose(double x, double y, double yaw, double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(yaw / 2);
  pose.orientation.w = std::cos(yaw / 2);
  return pose;
}

using concealer::isLocalizationConsistentWith;

TEST(LocalizationConsistency, accepts_rear_axle_offset_of_a_fresh_estimate)
{
  // The entity pose and the EKF's base_link differ by ~0.11 m along the heading.
  const auto expected = pose(190.7, -130.1, M_PI);
  EXPECT_TRUE(isLocalizationConsistentWith(expected, pose(190.81, -130.1, M_PI, 0.4), 1.0, 0.2));
}

TEST(LocalizationConsistency, rejects_the_previous_scenarios_final_pose)
{
  // Observed: route planned from (88.93, -131.13) when the ego spawned at (190.7, -130.1).
  EXPECT_FALSE(
    isLocalizationConsistentWith(pose(190.7, -130.1, M_PI), pose(88.93, -131.13, M_PI), 1.0, 0.2));
}

TEST(LocalizationConsistency, rejects_a_nearby_pose_with_the_wrong_heading)
{
  EXPECT_FALSE(isLocalizationConsistentWith(pose(0, 0, 0), pose(0.2, 0, M_PI / 2), 1.0, 0.2));
}

TEST(LocalizationConsistency, yaw_difference_wraps_around_pi)
{
  EXPECT_TRUE(
    isLocalizationConsistentWith(pose(0, 0, M_PI - 0.05), pose(0, 0, -M_PI + 0.05), 1.0, 0.2));
}

TEST(LocalizationConsistency, position_tolerance_is_horizontal_and_inclusive)
{
  EXPECT_TRUE(isLocalizationConsistentWith(pose(0, 0, 0), pose(0.6, 0.8, 0, 5.0), 1.0, 0.2));
  EXPECT_FALSE(isLocalizationConsistentWith(pose(0, 0, 0), pose(0.6, 0.81, 0), 1.0, 0.2));
}
