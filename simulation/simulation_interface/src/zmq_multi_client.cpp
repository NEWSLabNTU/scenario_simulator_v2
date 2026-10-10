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

#include <rclcpp/utilities.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <cstdlib>
#include <string>
namespace zeromq
{
namespace
{
/*
   Seconds from the environment variable `name`, else `default_seconds`, as milliseconds.

   SIMULATOR_RESPONSE_TIMEOUT (default 90 s) bounds every request but Initialize: a frame
   that ticks a server which stopped answering fails on carla-scenario-bridge's own 30 s
   RPC timeout (plus a 10 s apply_settings), so 90 s is twice that, and a simulator that
   died mid-scenario is reported in a minute and a half rather than seven minutes.

   SIMULATOR_INITIALIZE_TIMEOUT (default 300 s) bounds Initialize, which may wait for a
   restarting CARLA (bridge carla.reconnect_wait_seconds, 120) and then load another town
   (up to the bridge's 120 s map-load RPC timeout; 44 s measured for Town02 at Epic).
*/
auto timeoutMilliseconds(const char * name, double default_seconds) -> int
{
  if (const auto * value = std::getenv(name); value and *value) {
    return static_cast<int>(std::stod(value) * 1000);
  } else {
    return static_cast<int>(default_seconds * 1000);
  }
}
}  // namespace

MultiClient::MultiClient(
  const simulation_interface::TransportProtocol & protocol, const std::string & hostname,
  const unsigned int socket_port)
: protocol(protocol),
  hostname(hostname),
  context_(1),
  type_(zmq::socket_type::req),
  endpoint_(simulation_interface::getEndPoint(protocol, hostname, socket_port)),
  receive_timeout_ms_(timeoutMilliseconds("SIMULATOR_RESPONSE_TIMEOUT", 90)),
  initialize_timeout_ms_(timeoutMilliseconds("SIMULATOR_INITIALIZE_TIMEOUT", 300)),
  socket_(context_, type_)
{
  connect();
}

void MultiClient::connect()
{
  current_timeout_ms_ = -2;
  // Do not hold a dead peer's unsent requests on close.
  socket_.set(zmq::sockopt::linger, 0);
  socket_.connect(endpoint_);
}

void MultiClient::closeConnection()
{
  if (is_running) {
    is_running = false;
    socket_.close();
  }
}

MultiClient::~MultiClient() { closeConnection(); }

auto MultiClient::call(const simulation_api_schema::SimulationRequest & req)
  -> simulation_api_schema::SimulationResponse
{
  return call(req, req.has_initialize() ? initialize_timeout_ms_ : receive_timeout_ms_);
}

auto MultiClient::call(const simulation_api_schema::SimulationRequest & req, int timeout_ms)
  -> simulation_api_schema::SimulationResponse
{
  if (timeout_ms != current_timeout_ms_) {
    socket_.set(zmq::sockopt::rcvtimeo, timeout_ms > 0 ? timeout_ms : -1);
    current_timeout_ms_ = timeout_ms;
  }
  zmq::message_t message = toZMQ(req);
  socket_.send(message, zmq::send_flags::none);
  zmq::message_t buffer;
  if (not socket_.recv(buffer, zmq::recv_flags::none)) {
    /*
       A REQ socket whose reply never came cannot send again, so replace it: whatever
       handles the error -- or the next scenario in this process -- gets a usable client.
    */
    socket_.close();
    socket_ = zmq::socket_t(context_, type_);
    connect();
    THROW_SIMULATION_ERROR(
      "No response from the simulator at ", endpoint_, " within ", timeout_ms / 1000.0, " s (",
      req.has_initialize() ? "SIMULATOR_INITIALIZE_TIMEOUT" : "SIMULATOR_RESPONSE_TIMEOUT",
      "). It may have crashed or hung with this request outstanding, or not be running.");
  }
  return toProto<simulation_api_schema::SimulationResponse>(buffer);
}

auto MultiClient::call(const simulation_api_schema::InitializeRequest & request)
  -> simulation_api_schema::InitializeResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_initialize() = request;
    return call(sim_request).initialize();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::UpdateFrameRequest & request)
  -> simulation_api_schema::UpdateFrameResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_update_frame() = request;
    return call(sim_request).update_frame();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::UpdateStepTimeRequest & request)
  -> simulation_api_schema::UpdateStepTimeResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_update_step_time() = request;
    return call(sim_request).update_step_time();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::UpdateEntityGoalRequest & request)
  -> simulation_api_schema::UpdateEntityGoalResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_update_entity_goal() = request;
    return call(sim_request).update_entity_goal();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::SpawnVehicleEntityRequest & request)
  -> simulation_api_schema::SpawnVehicleEntityResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_spawn_vehicle_entity() = request;
    return call(sim_request).spawn_vehicle_entity();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::SpawnPedestrianEntityRequest & request)
  -> simulation_api_schema::SpawnPedestrianEntityResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_spawn_pedestrian_entity() = request;
    return call(sim_request).spawn_pedestrian_entity();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::SpawnMiscObjectEntityRequest & request)
  -> simulation_api_schema::SpawnMiscObjectEntityResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_spawn_misc_object_entity() = request;
    return call(sim_request).spawn_misc_object_entity();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::DespawnEntityRequest & request)
  -> simulation_api_schema::DespawnEntityResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_despawn_entity() = request;
    return call(sim_request).despawn_entity();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::UpdateEntityStatusRequest & request)
  -> simulation_api_schema::UpdateEntityStatusResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_update_entity_status() = request;
    return call(sim_request).update_entity_status();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::AttachImuSensorRequest & request)
  -> simulation_api_schema::AttachImuSensorResponse
{
  if (is_running) {
    auto simulation_request = simulation_api_schema::SimulationRequest();
    *simulation_request.mutable_attach_imu_sensor() = request;
    return call(simulation_request).attach_imu_sensor();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::AttachLidarSensorRequest & request)
  -> simulation_api_schema::AttachLidarSensorResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_attach_lidar_sensor() = request;
    return call(sim_request).attach_lidar_sensor();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::AttachDetectionSensorRequest & request)
  -> simulation_api_schema::AttachDetectionSensorResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_attach_detection_sensor() = request;
    return call(sim_request).attach_detection_sensor();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::AttachOccupancyGridSensorRequest & request)
  -> simulation_api_schema::AttachOccupancyGridSensorResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_attach_occupancy_grid_sensor() = request;
    return call(sim_request).attach_occupancy_grid_sensor();
  } else {
    return {};
  }
}

auto MultiClient::call(const simulation_api_schema::UpdateTrafficLightsRequest & request)
  -> simulation_api_schema::UpdateTrafficLightsResponse
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    *sim_request.mutable_update_traffic_lights() = request;
    return call(sim_request).update_traffic_lights();
  } else {
    return {};
  }
}

auto MultiClient::call(
  const simulation_api_schema::AttachPseudoTrafficLightDetectorRequest & request)
  -> simulation_api_schema::AttachPseudoTrafficLightDetectorResponse
{
  if (is_running) {
    auto simulation_request = simulation_api_schema::SimulationRequest();
    *simulation_request.mutable_attach_pseudo_traffic_light_detector() = request;
    return call(simulation_request).attach_pseudo_traffic_light_detector();
  } else {
    return {};
  }
}
}  // namespace zeromq
