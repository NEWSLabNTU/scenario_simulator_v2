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

#include <boost/range/adaptor/sliced.hpp>
#include <concealer/field_operator_application.hpp>
#include <concealer/is_package_exists.hpp>
#include <concealer/member_detector.hpp>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <tuple>
#include <exception>
#include <scenario_simulator_exception/exception.hpp>
#include <system_error>

namespace concealer
{
template <typename T>
auto toModuleType(const std::string & module_name)
{
  static const std::unordered_map<std::string, std::uint8_t> module_type_map = [&]() {
    std::unordered_map<std::string, std::uint8_t> module_type_map;

#define EMPLACE(IDENTIFIER)                                  \
  if constexpr (DetectStaticMember_##IDENTIFIER<T>::value) { \
    module_type_map.emplace(#IDENTIFIER, T::IDENTIFIER);     \
  }                                                          \
  static_assert(true)

    /*
       The following elements are in order of definition in the
       tier4_rtc_msgs/msg/Module.msg file. Of course, unordered_map doesn't
       preserve the insertion order, so the order itself doesn't matter.
    */
    EMPLACE(NONE);
    EMPLACE(LANE_CHANGE_LEFT);
    EMPLACE(LANE_CHANGE_RIGHT);
    EMPLACE(AVOIDANCE_LEFT);
    EMPLACE(AVOIDANCE_RIGHT);
    EMPLACE(GOAL_PLANNER);
    EMPLACE(START_PLANNER);
    EMPLACE(PULL_OUT);
    EMPLACE(TRAFFIC_LIGHT);
    EMPLACE(INTERSECTION);
    EMPLACE(INTERSECTION_OCCLUSION);
    EMPLACE(CROSSWALK);
    EMPLACE(BLIND_SPOT);
    EMPLACE(DETECTION_AREA);
    EMPLACE(NO_STOPPING_AREA);
    EMPLACE(OCCLUSION_SPOT);
    EMPLACE(EXT_REQUEST_LANE_CHANGE_LEFT);
    EMPLACE(EXT_REQUEST_LANE_CHANGE_RIGHT);
    EMPLACE(AVOIDANCE_BY_LC_LEFT);
    EMPLACE(AVOIDANCE_BY_LC_RIGHT);
    EMPLACE(NO_DRIVABLE_LANE);

#undef EMPLACE

    return module_type_map;
  }();

  if (const auto module_type = module_type_map.find(module_name);
      module_type == module_type_map.end()) {
    throw common::Error(
      "Unexpected module name for tier4_rtc_msgs::msg::Module: ", module_name, ".");
  } else {
    return module_type->second;
  }
}

// clang-format off
FieldOperatorApplication::FieldOperatorApplication(const pid_t pid, const bool managed)
: rclcpp::Node("concealer_user", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
  process_id(pid),
  managed(managed),
  time_limit(std::chrono::steady_clock::now() + std::chrono::seconds(common::getParameter<int>("initialize_duration"))),
  getAutowareState(managed, "/autoware/state", rclcpp::QoS(1), *this),
  getCommand(managed, "/control/command/control_cmd", rclcpp::QoS(1), *this),
  getCooperateStatusArray(managed, "/api/external/get/rtc_status", rclcpp::QoS(1), *this),
  getEmergencyState(managed, "/api/external/get/emergency", rclcpp::QoS(1), *this, [this](const auto & message) {
    if (message.emergency) {
      throw common::Error("Emergency state received");
    }
  }),
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  getLocalizationState(managed, "/api/localization/initialization_state", rclcpp::QoS(1).transient_local(), *this),
#endif
  getMrmState(managed, "/api/fail_safe/mrm_state", rclcpp::QoS(1), *this, [this](const auto & message) {
    auto state_name_of = [](auto state) constexpr {
      switch (state) {
        case MrmState::MRM_FAILED:
          return "MRM_FAILED";
        case MrmState::MRM_OPERATING:
          return "MRM_OPERATING";
        case MrmState::MRM_SUCCEEDED:
          return "MRM_SUCCEEDED";
        case MrmState::NORMAL:
          return "NORMAL";
        case MrmState::UNKNOWN:
          return "UNKNOWN";
        default:
          throw common::Error(
            "Unexpected autoware_adapi_v1_msgs::msg::MrmState::state, number: ", state);
      }
    };

    auto behavior_name_of = [](auto behavior) constexpr {
      if constexpr (DetectStaticMember_COMFORTABLE_STOP<MrmState>::value) {
        if (behavior == MrmState::COMFORTABLE_STOP) {
          return "COMFORTABLE_STOP";
        }
      }
      if constexpr (DetectStaticMember_EMERGENCY_STOP<MrmState>::value) {
        if (behavior == MrmState::EMERGENCY_STOP) {
          return "EMERGENCY_STOP";
        }
      }
      if constexpr (DetectStaticMember_NONE<MrmState>::value) {
        if (behavior == MrmState::NONE) {
          return "NONE";
        }
      }
      if constexpr (DetectStaticMember_UNKNOWN<MrmState>::value) {
        if (behavior == MrmState::UNKNOWN) {
          return "UNKNOWN";
        }
      }
      if constexpr (DetectStaticMember_PULL_OVER<MrmState>::value) {
        if (behavior == MrmState::PULL_OVER) {
          return "PULL_OVER";
        }
      }
      throw common::Error(
        "Unexpected autoware_adapi_v1_msgs::msg::MrmState::behavior, number: ", behavior);
    };

    minimum_risk_maneuver_state = state_name_of(message.state);
    minimum_risk_maneuver_behavior = behavior_name_of(message.behavior);
  }),
#if __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
  getOperationModeState(managed, "/api/operation_mode/state", rclcpp::QoS(1).transient_local(), *this),
#endif
  getPathWithLaneId(managed, "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", rclcpp::QoS(1), *this),
#if __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>)
  getRouteState(managed, "/api/routing/state", rclcpp::QoS(1).transient_local(), *this),
#endif
  getTurnIndicatorsCommand(managed, "/control/command/turn_indicators_cmd", rclcpp::QoS(1), *this),
  getKinematicState(managed, "/localization/kinematic_state", rclcpp::QoS(1), *this),
  requestClearRoute(managed, "/api/routing/clear_route", *this),
  requestCooperateCommands(managed, "/api/external/set/rtc_commands", *this),
  requestEngage(managed, "/api/external/set/engage", *this),
  requestInitialPose(managed, "/api/localization/initialize", *this, std::chrono::seconds(common::getParameter<int>("initialize_localization"))),
  // NOTE: routing takes a long time to return. But the specified duration is not decided by any technical reasons.
  requestSetRoute(managed, "/api/routing/set_route", *this, std::chrono::seconds(10)),
  requestSetRoutePoints(managed, "/api/routing/set_route_points", *this, std::chrono::seconds(10)),
  requestSetRtcAutoMode(managed, "/api/external/set/rtc_auto_mode", *this),
  requestSetVelocityLimit(managed, "/api/autoware/set/velocity_limit", *this),
  requestEnableAutowareControl(managed, "/api/operation_mode/enable_autoware_control", *this),
  requestChangeToStop(managed, "/api/operation_mode/change_to_stop", *this)
// clang-format on
{
  executor.add_node(get_node_base_interface());

  if (managed) {
    localization_scan_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/util/downsample/pointcloud", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {
        latest_localization_scan_stamp.store(rclcpp::Time(scan->header.stamp).nanoseconds());
        ++localization_scan_count;
      });
  }

  /*
     In case of reusing the same Autoware instance for multiple scenarios (launch_autoware:=False),
     we need to ensure that Autoware is in a safe STOP state before starting the next scenario.
     Without this, Autoware may start driving unexpectedly when the next scenario starts.
  */
  if (managed) {
    task_queue.delay([this] {
      /*
         To ensure that Autoware is in a safe state, request to change to stop.
         Ideally, we should check the operation state and only request if it's not already in STOP mode.
         TODO: Implement state check to avoid unnecessary requests (when LegacyAutowareState is being refactored).
      */
      requestChangeToStop(std::make_shared<ChangeOperationMode::Request>(), 30);
    });
  }
}

auto FieldOperatorApplication::requireManaged(const std::string & operation) const -> void
{
  if (not managed) {
    throw common::SemanticError(
      operation,
      " cannot be requested because the ego vehicle is not managed by scenario_simulator_v2 "
      "(the parameter managed_ego:=false was given). Scenarios that use ego autonomy actions "
      "require managed_ego:=true.");
  }
}

FieldOperatorApplication::~FieldOperatorApplication()
{
  /*
     Hand a reused Autoware (launch_autoware:=false) back in STOP. Without this the
     ego leaves the scenario engaged -- AUTONOMOUS, its EKF still moving at the speed
     the ego ended at -- and that is the state the next scenario's first /clock tick
     resumes. Every diagnostic that then blinks while localization is re-initialized
     on the new vehicle (a pose jump, an initial covariance, the sensor gap before the
     new ego's first frame) makes autonomous mode unavailable *while in autonomous
     mode*, which the MRM handler answers with an EMERGENCY_STOP: measured at every
     scenario start, before the scenario had asked Autoware for anything. The
     constructor's own change_to_stop comes too late to prevent it -- by then the mode
     is unavailable and the request is refused.

     The route is left alone: /api/routing/clear_route is refused here, because the
     routing API only learns of the STOP mode from a state message published on the
     next /clock tick, and the next scenario's localization initialization clears the
     route anyway.

     /clock has already stopped when this runs, so the mode is in place before
     Autoware's next tick. Bounded (the executor is spun here: nothing else spins this
     node any more) and best effort: a missing or unresponsive Autoware costs a few
     seconds and a log line, never the scenario result.
  */
  if (managed and not process_id) {
    if (requestChangeToStop.callOnce(
          std::make_shared<ChangeOperationMode::Request>(), executor, std::chrono::seconds(3))) {
      RCLCPP_INFO_STREAM(get_logger(), "Left Autoware in STOP for the next scenario.");
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Could not change Autoware to STOP at the end of the scenario; the next scenario "
        "starts in this one's operation mode.");
    }
  }

  if (process_id) {
    const auto sigset = [this]() {
      if (auto signal_set = sigset_t();
          sigemptyset(&signal_set) or sigaddset(&signal_set, SIGCHLD)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else if (auto error = pthread_sigmask(SIG_BLOCK, &signal_set, nullptr)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(error, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else {
        return signal_set;
      }
    }();

    const auto timeout = []() {
      auto timeout = timespec();
      timeout.tv_sec = common::getParameter<int>("sigterm_timeout", 5);
      timeout.tv_nsec = 0;
      return timeout;
    }();

    if (::kill(process_id, SIGINT); sigtimedwait(&sigset, nullptr, &timeout) < 0) {
      switch (errno) {
        case EINTR:
          /*
             The wait was interrupted by an unblocked, caught signal. It shall
             be documented in system documentation whether this error causes
             these functions to fail.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The wait for Autoware launch process termination was interrupted by an unblocked, "
            "caught signal.");
          break;

        case EAGAIN:
          /*
             No signal specified by set was generated within the specified
             timeout period.
          */
          RCLCPP_ERROR_STREAM(get_logger(), "Autoware launch process does not respond. Kill it.");
          killpg(process_id, SIGKILL);
          break;

        default:
        case EINVAL:
          /*
             The timeout argument specified a tv_nsec value less than zero or
             greater than or equal to 1000 million.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The parameter sigterm_timeout specified a value less than zero or greater than or "
            "equal to 1000 million.");
          break;
      }
    }

    if (int status = 0; waitpid(process_id, &status, 0) < 0) {
      if (errno == ECHILD) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Try to wait for the autoware process but it was already exited.");
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
      }
    }

    process_id = 0;
  }

  finalized.store(true);
}

auto FieldOperatorApplication::clearRoute() -> void
{
  requireManaged("clearRoute");

  task_queue.delay([this] {
    /*
       Since this service tends to be available long after the launch of
       Autoware, set the attempts_count to a high value. There is no technical
       basis for the number 30.
    */
    requestClearRoute(std::make_shared<ClearRoute::Request>(), 30);
  });
}

auto FieldOperatorApplication::enableAutowareControl() -> void
{
  requireManaged("enableAutowareControl");

  task_queue.delay([this]() {
    auto request = std::make_shared<ChangeOperationMode::Request>();
    requestEnableAutowareControl(request, 30);
  });
}

auto FieldOperatorApplication::engage() -> void
{
  requireManaged("engage");

  task_queue.delay([this]() {
    switch (const auto state = getLegacyAutowareState(); state.value) {
      default:
        throw common::AutowareError(
          "The simulator attempted to request Autoware to engage, but was aborted because "
          "Autoware's current state is ",
          state, ".");
      case LegacyAutowareState::initializing:
        // The initial pose has been sent but has not yet reached Autoware.
        waitForAutowareStateToBe(
          LegacyAutowareState::initializing, LegacyAutowareState::waiting_for_route);
        [[fallthrough]];
      case LegacyAutowareState::waiting_for_route:
        // The route has been sent but has not yet reached Autoware.
        waitForAutowareStateToBe(
          LegacyAutowareState::waiting_for_route, LegacyAutowareState::planning);
        [[fallthrough]];
      case LegacyAutowareState::planning:
        waitForAutowareStateToBe(
          LegacyAutowareState::planning, LegacyAutowareState::waiting_for_engage);
        [[fallthrough]];
      case LegacyAutowareState::waiting_for_engage:
        requestEngage(
          [&]() {
            auto request = std::make_shared<Engage::Request>();
            request->engage = true;
            return request;
          }(),
          30);
        waitForAutowareStateToBe(
          LegacyAutowareState::waiting_for_engage, LegacyAutowareState::driving);
        time_limit = std::decay_t<decltype(time_limit)>::max();
        break;
      case LegacyAutowareState::driving:
        break;
      case LegacyAutowareState::arrived_goal:
        // On a short route in a fast simulation, Autoware can complete the
        // drive before this queued engage task runs. Arrival implies the
        // engagement already happened; treating it as an error would fail a
        // scenario whose vehicle did exactly what was asked.
        time_limit = std::decay_t<decltype(time_limit)>::max();
        break;
    }
  });
}

auto FieldOperatorApplication::engageable() const -> bool
{
  task_queue.rethrow();
  return task_queue.empty() and
         getLegacyAutowareState().value == LegacyAutowareState::waiting_for_engage;
}

auto FieldOperatorApplication::engaged() const -> bool
{
  task_queue.rethrow();
  return task_queue.empty() and getLegacyAutowareState().value == LegacyAutowareState::driving;
}

auto FieldOperatorApplication::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  requireManaged("initialize");

  if (not std::exchange(initialized, true)) {
    task_queue.delay([this, initial_pose]() {
      switch (const auto state = getLegacyAutowareState(); state.value) {
        default:
          throw common::AutowareError(
            "The simulator attempted to initialize Autoware, but aborted because Autoware's "
            "current state is ",
            state, ".");
        case LegacyAutowareState::undefined:
          waitForAutowareStateToBe(
            LegacyAutowareState::undefined, LegacyAutowareState::initializing);
          [[fallthrough]];
        case LegacyAutowareState::initializing:
        case LegacyAutowareState::waiting_for_route: {
          /*
             The stamp of the newest localization estimate from before this
             initialization. Any estimate that can confirm the new pose must be
             stamped later. (Comparing with this node's clock instead would be
             wrong: this node runs on wall time while Autoware stamps with the
             /clock the simulator publishes.)
          */
          const auto stamp_before_initialization = getKinematicState().header.stamp;
          /*
             NDT aligns the initial pose against the newest scan it holds. Requested
             at once -- ~0.1 s after the ego spawns -- that scan still belongs to the
             vehicle the previous scenario despawned, possibly 200 m away, and the
             alignment answers with it: measured up to 1.6 m off and, once, facing
             backwards (yaw 0.04 for 3.14), which waitForLocalizationToReach then
             rightly refused. Wait for scans taken by this ego first; they cost well
             under a second.
          */
          waitForScansNewerThan(stamp_before_initialization);
          requestInitialPose(
            [&]() {
              auto request =
                std::make_shared<autoware_adapi_v1_msgs::srv::InitializeLocalization::Request>();
              request->pose.push_back([&]() {
                auto initial_pose_stamped = geometry_msgs::msg::PoseWithCovarianceStamped();
                /*
                   Autoware's time base, not this node's: the concealer runs on wall
                   time while Autoware runs on the simulator's /clock, so now() put
                   the initial pose ~1.79e9 s away from every stamp Autoware holds.
                   The newest scan, just waited for above, is in Autoware's time.
                */
                if (const auto scan_stamp = latest_localization_scan_stamp.load();
                    scan_stamp > 0) {
                  initial_pose_stamped.header.stamp = rclcpp::Time(scan_stamp, RCL_ROS_TIME);
                } else {
                  // No scan ever arrived (warned above); nothing better is known.
                  initial_pose_stamped.header.stamp = get_clock()->now();
                }
                initial_pose_stamped.header.frame_id = "map";
                initial_pose_stamped.pose.pose = initial_pose;
                return initial_pose_stamped;
              }());
              return request;
            }(),
            30);
          waitForAutowareStateToBe(
            LegacyAutowareState::initializing, LegacyAutowareState::waiting_for_route);
          /*
             waiting_for_route only says the localization pipeline was
             re-activated, not that it has published from the new pose: the EKF
             publishes on /clock ticks, and until its first tick after
             activation /localization/kinematic_state still holds the previous
             estimate. With a long-lived Autoware reused across scenarios that
             is where the previous scenario ended, and mission_planner, which
             takes the route's start from the latest kinematic_state, would plan
             from there. Hold the task queue (and so any plan() queued after
             this) until localization agrees with the pose just given.
          */
          waitForLocalizationToReach(initial_pose, stamp_before_initialization);
          break;
        }
      }
    });
  }
}

auto FieldOperatorApplication::waitForScansNewerThan(const builtin_interfaces::msg::Time & stamp)
  -> void
{
  /*
     Two scans, not one: the first after a spawn can be a partial sweep, and the
     second guarantees NDT has finished taking in the first. 10 s of wall time is
     the same margin waitForLocalizationToReach allows for a /clock that runs
     slower than wall time.
  */
  constexpr std::uint64_t scans_required = 2;
  const auto reference = rclcpp::Time(stamp).nanoseconds();
  const auto deadline =
    std::min(std::chrono::steady_clock::now() + std::chrono::seconds(10), time_limit);
  std::uint64_t fresh = 0;
  for (auto counted = localization_scan_count.load(); not finalized.load();) {
    if (const auto count = localization_scan_count.load(); count != counted) {
      counted = count;
      if (latest_localization_scan_stamp.load() > reference and ++fresh >= scans_required) {
        return;
      }
    }
    if (deadline <= std::chrono::steady_clock::now()) {
      RCLCPP_WARN_STREAM(
        get_logger(), "NDT received " << fresh << " scan(s) from the new ego within 10 s; "
                                          "initializing localization anyway.");
      return;
    }
    rclcpp::GenericRate<std::chrono::steady_clock>(std::chrono::milliseconds(20)).sleep();
  }
}

auto isLocalizationConsistentWith(
  const geometry_msgs::msg::Pose & expected, const geometry_msgs::msg::Pose & actual,
  const double position_tolerance, const double yaw_tolerance) -> bool
{
  auto yaw_of = [](const auto & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  const auto yaw_difference = std::remainder(
    yaw_of(actual.orientation) - yaw_of(expected.orientation), 2.0 * M_PI);
  return std::hypot(actual.position.x - expected.position.x,
                    actual.position.y - expected.position.y) <= position_tolerance and
         std::abs(yaw_difference) <= yaw_tolerance;
}

auto FieldOperatorApplication::waitForLocalizationToReach(
  const geometry_msgs::msg::Pose & initial_pose,
  const builtin_interfaces::msg::Time & stamp_before_initialization) -> void
{
  /*
     1.0 m: the estimate is of base_link (the rear axle), which lies about
     0.11 m from the pose an external simulator reports for the entity, and a
     freshly initialized EKF settles within centimetres of its initial pose. A
     stale estimate is wherever the previous scenario ended, typically tens of
     metres away. 1 m is well above the former and below half a lane width, so a
     route started from any accepted estimate starts in the right lane.

     0.2 rad (~11.5 deg): a stationary, just-initialized ego matches the
     commanded yaw to well under a degree; the bound only has to reject an
     estimate from a different pose.

     10 s: the EKF publishes once per 0.1 s of /clock, and /clock can run
     slower than wall time on a loaded host (0.3x measured), so a fresh
     estimate normally arrives within a second of wall time. 10 s is an order
     of magnitude of margin while still failing well before the scenario's own
     time limit.
  */
  constexpr auto position_tolerance = 1.0;
  constexpr auto yaw_tolerance = 0.2;
  constexpr auto timeout = std::chrono::seconds(10);

  auto newer_than = [](const auto & a, const auto & b) {
    return std::tie(a.sec, a.nanosec) > std::tie(b.sec, b.nanosec);
  };

  auto describe = [](const geometry_msgs::msg::Pose & pose) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "(x " << pose.position.x << ", y "
       << pose.position.y << ", yaw "
       << std::atan2(
            2.0 * (pose.orientation.w * pose.orientation.z +
                   pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                         pose.orientation.z * pose.orientation.z))
       << ")";
    return ss.str();
  };

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = std::min(start + timeout, time_limit);

  for (auto stale_checks = 0; not finalized.load(); ++stale_checks) {
    const auto estimate = getKinematicState();
    const auto fresh = newer_than(estimate.header.stamp, stamp_before_initialization);
    if (fresh and isLocalizationConsistentWith(
                    initial_pose, estimate.pose.pose, position_tolerance, yaw_tolerance)) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Localization reached the initial pose "
          << describe(initial_pose) << " at " << describe(estimate.pose.pose) << " after "
          << std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start)
               .count()
          << " ms (" << (stale_checks == 0 ? "no wait" : "waited") << ", " << stale_checks
          << " stale checks); a route may now be requested.");
      return;
    } else if (deadline <= std::chrono::steady_clock::now()) {
      throw common::AutowareError(
        "Simulator initialized localization at ", describe(initial_pose),
        ", but /localization/kinematic_state did not reach it within ",
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start)
          .count(),
        " ms (tolerance ", position_tolerance, " m, ", yaw_tolerance,
        " rad). The latest estimate is ", describe(estimate.pose.pose), " stamped ",
        [&]() {
          std::stringstream ss;
          ss << estimate.header.stamp.sec << "." << std::setw(9) << std::setfill('0')
             << estimate.header.stamp.nanosec;
          return ss.str();
        }(),
        fresh ? "" : " (not newer than the initialization)",
        ". Refusing to request a route that would be planned from that pose.");
    } else {
      rclcpp::GenericRate<std::chrono::steady_clock>(std::chrono::milliseconds(20)).sleep();
    }
  }
}

auto FieldOperatorApplication::plan(
  const std::vector<geometry_msgs::msg::PoseStamped> & route, const bool allow_goal_modification)
  -> void
{
  assert(not route.empty());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (route.size() > 1) {
    std::transform(
      route.begin(), route.end() - 1, std::back_inserter(waypoints),
      [](const auto & stamped_pose) { return stamped_pose.pose; });
  }

  RouteOption route_option;
  if (DetectMember_allow_goal_modification<RouteOption>::value) {
    route_option.allow_goal_modification = allow_goal_modification;
  }

  plan(route.back().pose, waypoints, route_option);
}

template <
  typename Request, typename Waypoint,
  typename = std::enable_if_t<
    std::is_same_v<Request, autoware_adapi_v1_msgs::srv::SetRoutePoints::Request> ||
    std::is_same_v<Request, autoware_adapi_v1_msgs::srv::SetRoute::Request> > >
static auto make(
  const geometry_msgs::msg::Pose & goal, const std::vector<Waypoint> & waypoints,
  const FieldOperatorApplication::RouteOption & option) -> std::shared_ptr<Request>
{
  auto request = std::make_shared<Request>();

  request->header.frame_id = "map";
  request->goal = goal;

  if constexpr (std::is_same_v<Request, autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>) {
    request->waypoints.assign(waypoints.begin(), waypoints.end());
  } else if constexpr (std::is_same_v<Request, autoware_adapi_v1_msgs::srv::SetRoute::Request>) {
    request->segments.assign(waypoints.begin(), waypoints.end());
  }

  /*
     NOTE: The autoware_adapi_v1_msgs::srv::SetRoutePoints::Request
     type was created on 2022/09/05 [1], and the
     autoware_adapi_v1_msgs::msg::Option type data member was added
     to the autoware_adapi_v1_msgs::srv::SetRoutePoints::Request type
     on 2023/04/12 [2]. Therefore, we cannot expect
     autoware_adapi_v1_msgs::srv::SetRoutePoints::Request to always
     have a data member `option`.

     [1] https://github.com/autowarefoundation/autoware_adapi_msgs/commit/805f8ebd3ca24564844df9889feeaf183101fbef
     [2] https://github.com/autowarefoundation/autoware_adapi_msgs/commit/cf310bd038673b6cbef3ae3b61dfe607212de419
  */
  if constexpr (
    DetectMember_option<Request>::value and
    DetectMember_allow_goal_modification<decltype(std::declval<Request>().option)>::value) {
    request->option.allow_goal_modification = option.allow_goal_modification;
  }

  return request;
}

auto FieldOperatorApplication::plan(
  const geometry_msgs::msg::Pose & goal, const std::vector<geometry_msgs::msg::Pose> & waypoints,
  const RouteOption & option) -> void
{
  requireManaged("plan");

  task_queue.delay([this, goal, waypoints, option]() {
    switch (const auto state = getLegacyAutowareState(); state.value) {
      default:
        throw common::AutowareError(
          "The simulator attempted to send a goal to Autoware, but was aborted because Autoware's "
          "current state is ",
          state, ".");
      case LegacyAutowareState::initializing:
        // The initial pose has been sent but has not yet reached Autoware.
      case LegacyAutowareState::arrived_goal:
        waitForAutowareStateToBe(state, LegacyAutowareState::waiting_for_route);
        [[fallthrough]];
      case LegacyAutowareState::waiting_for_route:
        requestSetRoutePoints(make<SetRoutePoints::Request>(goal, waypoints, option), 30);
        waitForAutowareStateToBe(
          LegacyAutowareState::waiting_for_route, LegacyAutowareState::planning);
        waitForAutowareStateToBe(
          LegacyAutowareState::planning, LegacyAutowareState::waiting_for_engage);
        break;
    }
  });
}

auto FieldOperatorApplication::plan(
  const geometry_msgs::msg::Pose & goal,
  const std::vector<autoware_adapi_v1_msgs::msg::RouteSegment> & waypoints,
  const RouteOption & option) -> void
{
  requireManaged("plan");

  task_queue.delay([this, goal, waypoints, option]() {
    switch (const auto state = getLegacyAutowareState(); state.value) {
      default:
        throw common::AutowareError(
          "The simulator attempted to send a goal to Autoware, but was aborted because Autoware's "
          "current state is ",
          state, ".");
      case LegacyAutowareState::initializing:
        // The initial pose has been sent but has not yet reached Autoware.
      case LegacyAutowareState::arrived_goal:
        waitForAutowareStateToBe(state, LegacyAutowareState::waiting_for_route);
        [[fallthrough]];
      case LegacyAutowareState::waiting_for_route:
        requestSetRoute(make<SetRoute::Request>(goal, waypoints, option), 30);
        waitForAutowareStateToBe(
          LegacyAutowareState::waiting_for_route, LegacyAutowareState::planning);
        waitForAutowareStateToBe(
          LegacyAutowareState::planning, LegacyAutowareState::waiting_for_engage);
        break;
    }
  });
}

auto FieldOperatorApplication::requestAutoModeForCooperation(
  const std::string & module_name, bool enable) -> void
{
  requireManaged("requestAutoModeForCooperation");

  /*
     The implementation of this function will not work properly if the
     `rtc_auto_mode_manager` package is present.
  */
  if (not isPackageExists("rtc_auto_mode_manager")) {
    task_queue.delay([this, module_name, enable]() {
      auto request = std::make_shared<AutoModeWithModule::Request>();
      request->module.type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name);
      request->enable = enable;
      /*
         We attempt to resend the service up to 30 times, but this number of
         times was determined by heuristics, not for any technical reason.
      */
      requestSetRtcAutoMode(request, 30);
    });
  } else {
    throw common::Error(
      "FieldOperatorApplication::requestAutoModeForCooperation is not supported in this "
      "environment, because rtc_auto_mode_manager is present.");
  }
}

auto FieldOperatorApplication::sendCooperateCommand(
  const std::string & module_name, const std::string & command) -> void
{
  requireManaged("sendCooperateCommand");

  const auto command_type = [&]() {
    if (command == "ACTIVATE") {
      return tier4_rtc_msgs::msg::Command::ACTIVATE;
    } else if (command == "DEACTIVATE") {
      return tier4_rtc_msgs::msg::Command::DEACTIVATE;
    } else {
      throw common::Error("Unexpected command for tier4_rtc_msgs::msg::Command: ", command, ".");
    }
  }();

  /*
     NOTE: Used cooperate statuses will be deleted correctly in Autoware side
     and provided via topic update. But, their update rate (typ. 10Hz) is lower
     than the one of scenario_simulator_v2. So, we need to check cooperate
     statuses if they are used or not in scenario_simulator_v2 side to avoid
     sending the same cooperate command when sending multiple commands between
     updates of cooperate statuses.
  */
  static std::vector<tier4_rtc_msgs::msg::CooperateStatus> used_cooperate_statuses;

  auto is_used_cooperate_status = [](const auto & cooperate_status) {
    return std::find_if(
             used_cooperate_statuses.begin(), used_cooperate_statuses.end(),
             [&cooperate_status](const auto & used_cooperate_status) {
               return used_cooperate_status.module == cooperate_status.module &&
                      used_cooperate_status.uuid == cooperate_status.uuid &&
                      used_cooperate_status.command_status.type ==
                        cooperate_status.command_status.type;
             }) != used_cooperate_statuses.end();
  };

  auto is_valid_cooperate_status =
    [](const auto & cooperate_status, auto command_type, auto module_type) {
      /**
         The finish_distance filter is set to over -20.0, because some valid rtc
         statuses has negative finish_distance due to the errors of localization or
         numerical calculation. This threshold is advised by a member of TIER IV
         planning and control team.

         The difference in the variable referred as a distance is the impact of the
         message specification changes in the following URL. This was also decided
         after consulting with a member of TIER IV planning and control team. ref:
         https://github.com/tier4/tier4_autoware_msgs/commit/8b85e6e43aa48cf4a439c77bf4bf6aee2e70c3ef
      */
      if constexpr (DetectMember_distance<tier4_rtc_msgs::msg::CooperateStatus>::value) {
        return cooperate_status.module.type == module_type &&
               command_type != cooperate_status.command_status.type &&
               cooperate_status.distance >= -20.0;
      } else {
        return cooperate_status.module.type == module_type &&
               command_type != cooperate_status.command_status.type &&
               cooperate_status.finish_distance >= -20.0;
      }
    };

  const auto cooperate_status_array = getCooperateStatusArray();

  if (const auto cooperate_status = std::find_if(
        cooperate_status_array.statuses.begin(), cooperate_status_array.statuses.end(),
        [&, module_type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name)](
          const auto & cooperate_status) {
          return is_valid_cooperate_status(cooperate_status, command_type, module_type) &&
                 not is_used_cooperate_status(cooperate_status);
        });
      cooperate_status == cooperate_status_array.statuses.end()) {
    std::stringstream what;
    what
      << "Failed to send a cooperate command: Cannot find a valid request to cooperate for module "
      << std::quoted(module_name) << " and command " << std::quoted(command) << ". "
      << "Please check if the situation is such that the request occurs when sending.";
    throw common::Error(what.str());
  } else {
    tier4_rtc_msgs::msg::CooperateCommand cooperate_command;
    cooperate_command.module = cooperate_status->module;
    cooperate_command.uuid = cooperate_status->uuid;
    cooperate_command.command.type = command_type;

    auto request = std::make_shared<tier4_rtc_msgs::srv::CooperateCommands::Request>();
    request->stamp = cooperate_status_array.stamp;
    request->commands.push_back(cooperate_command);

    task_queue.delay([this, request]() { requestCooperateCommands(request, 30); });

    used_cooperate_statuses.push_back(*cooperate_status);
  }
}

auto FieldOperatorApplication::setVelocityLimit(double velocity_limit) -> void
{
  /*
     Inert rather than fatal when unmanaged, for the same reason as the interpreter's
     automatic engage: this is not an autonomy action a scenario author asked for.
     `applyAssignControllerAction` calls it unconditionally for every entity carrying a
     controller, taking `maxSpeed` from the controller properties, so every ego reaches
     here during Init. Fast-failing it killed every unmanaged run before the storyboard
     started -- observed as the ego spawning and being despawned 0.4 s later.

     The limit still applies where it can: `EgoEntity::setVelocityLimit` records it in
     `behavior_parameter_` before calling this, and that is the simulator-side effect. Only
     the forward to Autoware's service is skipped, which is exactly what an inert
     application means.
  */
  if (not managed) {
    return;
  }

  task_queue.delay([this, velocity_limit]() {
    auto request = std::make_shared<SetVelocityLimit::Request>();
    request->velocity = velocity_limit;
    /*
       We attempt to resend the service up to 30 times, but this number of
       times was determined by heuristics, not for any technical reason.
    */
    requestSetVelocityLimit(request, 30);
  });
}

auto FieldOperatorApplication::getLegacyAutowareState() const -> LegacyAutowareState
{
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>) and \
    __has_include(<autoware_adapi_v1_msgs/msg/route_state.hpp>) and \
    __has_include(<autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>)
  return LegacyAutowareState(
    getLocalizationState(), getRouteState(), getOperationModeState(), now());
#else
  return LegacyAutowareState(getAutowareState());
#endif
}

auto FieldOperatorApplication::spinSome() -> void
{
  task_queue.rethrow();

  if (rclcpp::ok()) {
    if (process_id) {
      auto status = 0;
      if (const auto id = waitpid(process_id, &status, WNOHANG); id < 0) {
        switch (errno) {
          case ECHILD:
            process_id = 0;
            throw common::AutowareError("Autoware process is already terminated");
          default:
            RCLCPP_ERROR_STREAM(
              get_logger(), std::system_error(errno, std::system_category()).what());
            std::exit(EXIT_FAILURE);
        }
      } else if (0 < id) {
        if (WIFEXITED(status)) {
          process_id = 0;
          throw common::AutowareError(
            "Autoware process is unintentionally exited. exit code: ", WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          process_id = 0;
          throw common::AutowareError("Autoware process is killed. signal is ", WTERMSIG(status));
        }
      }
    }

    executor.spin_some();
  }
}
}  // namespace concealer
