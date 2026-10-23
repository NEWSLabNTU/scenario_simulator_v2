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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/environment.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Environment::Environment(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, local()), scope),
  parameter_declarations(
    readElement<ParameterDeclarations>("ParameterDeclarations", node, local())),
  time_of_day(
    node.child("TimeOfDay") ? std::make_optional<TimeOfDay>(node.child("TimeOfDay"), local())
                            : std::nullopt),
  weather(
    node.child("Weather") ? std::make_optional<Weather>(node.child("Weather"), local())
                          : std::nullopt),
  road_condition(
    node.child("RoadCondition")
      ? std::make_optional<RoadCondition>(node.child("RoadCondition"), local())
      : std::nullopt)
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
