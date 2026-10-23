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

#include <boost/lexical_cast.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/environment.hpp>
#include <openscenario_interpreter/syntax/environment_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
EnvironmentAction::EnvironmentAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node, {
      { "Environment",      [&](const auto & node) { return make<Environment>(node, scope);        } },
      { "CatalogReference", [&](const auto & node) { return CatalogReference(node, scope).make();  } },
    }))
// clang-format on
{
}

auto EnvironmentAction::accomplished() noexcept -> bool { return true; }

auto EnvironmentAction::endsImmediately() noexcept -> bool { return true; }

auto EnvironmentAction::run() -> void {}

/*
   The simulator is told the road condition; the time of day and the weather are not sent
   yet. A RoadCondition is a whole road state: it replaces the last one, properties included.
*/
auto EnvironmentAction::start() const -> void
{
  const auto & environment = as<Environment>();
  if (not environment.has_road_condition) {
    return;
  }
  simulation_api_schema::UpdateEnvironmentRequest request;
  auto & road_condition = *request.mutable_road_condition();
  road_condition.set_friction_scale_factor(environment.road_condition.friction_scale_factor);
  road_condition.set_wetness(boost::lexical_cast<String>(environment.road_condition.wetness));
  for (const auto & [name, property] : environment.road_condition.properties.properties) {
    (*road_condition.mutable_properties())[name] = property.value;
  }
  applyEnvironmentAction(request);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
