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
   The simulator is told the time of day, the weather and the road condition the Environment
   gives; one not given does not change. A RoadCondition is a whole road state: it replaces
   the last one, properties included. Within a Weather, each attribute and element not given
   is unchanged.
*/
auto EnvironmentAction::start() const -> void
{
  const auto & environment = as<Environment>();
  simulation_api_schema::UpdateEnvironmentRequest request;

  if (environment.time_of_day) {
    auto & time_of_day = *request.mutable_time_of_day();
    time_of_day.set_date_time(environment.time_of_day->date_time);
    time_of_day.set_animation(environment.time_of_day->animation);
  }

  if (environment.weather) {
    const auto & given = *environment.weather;
    auto & weather = *request.mutable_weather();
    if (given.fractional_cloud_cover) {
      weather.set_fractional_cloud_cover(boost::lexical_cast<String>(*given.fractional_cloud_cover));
    }
    if (given.cloud_state) {
      weather.set_cloud_state(*given.cloud_state);
    }
    if (given.sun) {
      auto & sun = *weather.mutable_sun();
      sun.set_azimuth(given.sun->azimuth);
      sun.set_elevation(given.sun->elevation);
      // illuminance, or the deprecated intensity (the same quantity, lx)
      sun.set_illuminance(given.sun->illuminance > 0 ? given.sun->illuminance : given.sun->intensity);
    }
    if (given.fog) {
      weather.mutable_fog()->set_visual_range(given.fog->visual_range);
    }
    if (given.precipitation) {
      auto & precipitation = *weather.mutable_precipitation();
      precipitation.set_precipitation_type(
        boost::lexical_cast<String>(given.precipitation->precipitation_type));
      /*
         precipitationIntensity is mm/h; the deprecated intensity is [0..1], taken here as a
         fraction of heavy rain, 25 mm/h.
      */
      precipitation.set_precipitation_intensity(
        given.precipitation->precipitation_intensity > 0
          ? double(given.precipitation->precipitation_intensity)
          : given.precipitation->intensity * 25.0);
    }
    if (given.wind) {
      weather.mutable_wind()->set_direction(given.wind->direction);
      weather.mutable_wind()->set_speed(given.wind->speed);
    }
    if (given.temperature) {
      weather.set_has_temperature(true);
      weather.set_temperature(*given.temperature);
    }
    if (given.atmospheric_pressure) {
      weather.set_has_atmospheric_pressure(true);
      weather.set_atmospheric_pressure(*given.atmospheric_pressure);
    }
  }

  if (environment.road_condition) {
    auto & road_condition = *request.mutable_road_condition();
    road_condition.set_friction_scale_factor(environment.road_condition->friction_scale_factor);
    road_condition.set_wetness(boost::lexical_cast<String>(environment.road_condition->wetness));
    for (const auto & [name, property] : environment.road_condition->properties.properties) {
      (*road_condition.mutable_properties())[name] = property.value;
    }
  }

  if (request.has_time_of_day() or request.has_weather() or request.has_road_condition()) {
    applyEnvironmentAction(request);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
