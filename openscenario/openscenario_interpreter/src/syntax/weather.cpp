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
#include <openscenario_interpreter/syntax/weather.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
namespace
{
template <typename T>
auto readOptionalElement(const char * name, const pugi::xml_node & node, Scope & scope)
  -> std::optional<T>
{
  if (const auto child = node.child(name)) {
    return std::make_optional<T>(child, scope);
  } else {
    return std::nullopt;
  }
}
}  // namespace

Weather::Weather(const pugi::xml_node & node, Scope & scope)
: atmospheric_pressure(readAttribute<Double>("atmosphericPressure", node, scope, std::nullopt)),
  temperature(readAttribute<Double>("temperature", node, scope, std::nullopt)),
  cloud_state(readAttribute<String>("cloudState", node, scope, std::nullopt)),
  fractional_cloud_cover(
    readAttribute<FractionalCloudCover>("fractionalCloudCover", node, scope, std::nullopt)),
  sun(readOptionalElement<Sun>("Sun", node, scope)),
  fog(readOptionalElement<Fog>("Fog", node, scope)),
  precipitation(readOptionalElement<Precipitation>("Precipitation", node, scope)),
  wind(readOptionalElement<Wind>("Wind", node, scope)),
  dome_image(readOptionalElement<DomeImage>("DomeImage", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/Weather.html

  if (atmospheric_pressure and
      not(80000 <= *atmospheric_pressure and *atmospheric_pressure <= 120000)) {
    THROW_SYNTAX_ERROR(
      std::quoted("Weather::atmosphericPressure"), "is out of range [80000...120000]");
  }

  if (temperature and not(170 <= *temperature and *temperature <= 340)) {
    THROW_SYNTAX_ERROR(std::quoted("Weather::temperature"), "is out of range [170...340]");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
