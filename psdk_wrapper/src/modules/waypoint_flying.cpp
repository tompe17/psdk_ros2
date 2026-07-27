/**
 * @file waypoint_flying.cpp
 *
 * @brief Waypoint flying module implementation. This module is responsible for
 * handling the waypoint flying (V2) commands.
 *
 * @author Tommy Persson
 * Contact: tommy.persson@liu.se
 *
 */

#include "psdk_wrapper/modules/waypoint_flying.hpp"

#include <unistd.h>

#include <functional>

#include "dji_error.h"
#include "dji_waypoint_v2.h"

// psdk_ros2::WaypointFlyingModule *wfm_pointer;

namespace psdk_ros2
{

WaypointFlyingModule::WaypointFlyingModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating WaypointFlyingModule");
}

WaypointFlyingModule::~WaypointFlyingModule()
{
  RCLCPP_INFO(get_logger(), "Destroying WaypointFlyingModule");
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring WaypointFlyingModule");

  state_push_publisher =
      this->create_publisher<psdk_interfaces::msg::WaypointV2MissionStatePush>(
          "psdk_ros2/waypointV2_mission_state", 10);
  event_push_publisher =
      this->create_publisher<psdk_interfaces::msg::WaypointV2MissionEventPush>(
          "psdk_ros2/waypointV2_mission_event", 10);

  subscribe_waypoint_v2_event_service =
      this->create_service<psdk_interfaces::srv::SubscribeWaypointV2Event>(
          "psdk_ros2/waypointV2_subscribeMissionEvent",
          std::bind(&WaypointFlyingModule::subscribe_waypoint_v2_event_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  subscribe_waypoint_v2_state_service =
      this->create_service<psdk_interfaces::srv::SubscribeWaypointV2State>(
          "psdk_ros2/waypointV2_subscribeMissionState",
          std::bind(&WaypointFlyingModule::subscribe_waypoint_v2_state_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  pause_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::PauseWaypointV2Mission>(
          "psdk_ros2/waypointV2_pauseMission",
          std::bind(&WaypointFlyingModule::pause_waypoint_v2_mission_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  resume_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::ResumeWaypointV2Mission>(
          "psdk_ros2/waypointV2_resumeMission",
          std::bind(&WaypointFlyingModule::resume_waypoint_v2_mission_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  stop_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::StopWaypointV2Mission>(
          "psdk_ros2/waypointV2_stopMission",
          std::bind(&WaypointFlyingModule::stop_waypoint_v2_mission_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  start_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::StartWaypointV2Mission>(
          "psdk_ros2/waypointV2_startMission",
          std::bind(&WaypointFlyingModule::start_waypoint_v2_mission_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  download_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::DownloadWaypointV2Mission>(
          "psdk_ros2/waypointV2_downloadMission",
          std::bind(
              &WaypointFlyingModule::download_waypoint_v2_mission_callback,
              this, std::placeholders::_1, std::placeholders::_2));
  upload_waypoint_v2_mission_service =
      this->create_service<psdk_interfaces::srv::UploadWaypointV2Mission>(
          "psdk_ros2/waypointV2_uploadMission",
          std::bind(&WaypointFlyingModule::upload_waypoint_v2_mission_callback,
                    this, std::placeholders::_1, std::placeholders::_2));
  generate_waypoint_v2_action_service =
      this->create_service<psdk_interfaces::srv::GenerateWaypointV2Action>(
          "psdk_ros2/waypointV2_generateActions",
          std::bind(&WaypointFlyingModule::generate_waypoint_v2_action_callback,
                    this, std::placeholders::_1, std::placeholders::_2));

  init_waypoint_v2_setting_service =
      this->create_service<psdk_interfaces::srv::InitWaypointV2Setting>(
          "psdk_ros2/waypointV2_initSetting",
          std::bind(&WaypointFlyingModule::init_waypoint_v2_setting_callback,
                    this, std::placeholders::_1, std::placeholders::_2));

  upload_waypoint_v2_action_service =
      this->create_service<psdk_interfaces::srv::UploadWaypointV2Action>(
          "psdk_ros2/waypointV2_uploadActions",
          std::bind(&WaypointFlyingModule::upload_waypoint_v2_action_callback,
                    this, std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating WaypointFlyingModule");

  // subscribe always
  subscribe_waypoint_v2_mission_event();
  subscribe_waypoint_v2_mission_state();

  return CallbackReturn::SUCCESS;
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating WaypointFlyingModule");
  return CallbackReturn::SUCCESS;
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up WaypointFlyingModule");
#if 0
  // ROS 2 subscribers
  flight_control_generic_sub_.reset();
  flight_control_position_yaw_sub_.reset();
  flight_control_velocity_yawrate_sub_.reset();
  flight_control_body_velocity_yawrate_sub_.reset();
  flight_control_rollpitch_yawrate_thrust_sub_.reset();

  // ROS 2 services
  set_home_from_gps_srv_.reset();
  set_home_from_current_location_srv_.reset();
  set_go_home_altitude_srv_.reset();
  get_go_home_altitude_srv_.reset();
  start_go_home_srv_.reset();
  cancel_go_home_srv_.reset();
  obtain_ctrl_authority_srv_.reset();
  release_ctrl_authority_srv_.reset();
  turn_on_motors_srv_.reset();
  turn_off_motors_srv_.reset();
  takeoff_srv_.reset();
  land_srv_.reset();
  cancel_landing_srv_.reset();
  start_confirm_landing_srv_.reset();
  start_force_landing_srv_.reset();
  set_horizontal_vo_obstacle_avoidance_srv_.reset();
  set_horizontal_radar_obstacle_avoidance_srv_.reset();
  set_upwards_vo_obstacle_avoidance_srv_.reset();
  set_upwards_radar_obstacle_avoidance_srv_.reset();
  set_downwards_vo_obstacle_avoidance_srv_.reset();
  get_horizontal_vo_obstacle_avoidance_srv_.reset();
  get_upwards_vo_obstacle_avoidance_srv_.reset();
  get_upwards_radar_obstacle_avoidance_srv_.reset();
  get_downwards_vo_obstacle_avoidance_srv_.reset();
  get_horizontal_radar_obstacle_avoidance_srv_.reset();
#endif
  return CallbackReturn::SUCCESS;
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting down WaypointFlyingModule");
  return CallbackReturn::SUCCESS;
}

bool
WaypointFlyingModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_INFO(get_logger(),
                "WaypointFlyingModule already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating WaypointFlyingModule");
  // wfm_pointer = this;

  T_DjiReturnCode resinit = DjiWaypointV2_Init();
  print_return_code("DjiWaypointV2_Init result: ", resinit);
  // print_return_code("Init mission result: ", resinit);

  if (resinit > 0)
  {
    is_module_initialized_ = false;
    return false;
  }

  is_module_initialized_ = true;
  return true;
}

bool
WaypointFlyingModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing WaypointFlyingModule");
#if 0
  T_DjiReturnCode return_code = DjiHmsManager_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the HMS module. Error code: %ld",
                 return_code);
    return false;
  }
#endif
  is_module_initialized_ = false;
  return true;
}

void
WaypointFlyingModule::subscribe_waypoint_v2_event_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::SubscribeWaypointV2Event::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2Event::Response>
        res) const
{
  (void)req;

  res->result = subscribe_waypoint_v2_mission_event();

  if (res->result)
  {
    RCLCPP_INFO(get_logger(),
                "Successfully registered waypoint mission event callback");
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to register waypoint mission event callback");
  }
}

T_DjiReturnCode
mission_event_callback(T_DjiWaypointV2MissionEventPush eventData)
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mission_event_callback");

  psdk_interfaces::msg::WaypointV2MissionEventPush msg;
  msg.event = eventData.event;
  msg.f_c_timestamp = eventData.FCTimestamp;
  msg.interrupt_reason = 0;
  msg.recover_process = 0;
  msg.finish_reason = 0;
  msg.waypoint_index = 0;
  msg.current_mission_exec_num = 0;
  msg.finished_all_exec_num = 0;

  global_wp_ptr_->event_push_publisher->publish(msg);

  return 0;
}

T_DjiReturnCode
mission_state_callback(T_DjiWaypointV2MissionStatePush stateData)
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mission_state_callback");

  psdk_interfaces::msg::WaypointV2MissionStatePush msg;
  msg.common_data_version = 0;
  msg.common_data_len = 0;
  msg.cur_waypoint_index = stateData.curWaypointIndex;
  msg.state = stateData.state;  // 0x1 mission prepared; 0x2 enter mission
  msg.velocity = stateData.velocity;

  global_wp_ptr_->state_push_publisher->publish(msg);

  return 0;
}

bool
WaypointFlyingModule::subscribe_waypoint_v2_mission_state()
{
  T_DjiReturnCode ret =
      DjiWaypointV2_RegisterMissionStateCallback(&mission_state_callback);

  global_wp_ptr_->print_return_code("Subscribe mission state result: ", ret);

  return ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool
WaypointFlyingModule::subscribe_waypoint_v2_mission_event()
{
  T_DjiReturnCode ret =
      DjiWaypointV2_RegisterMissionEventCallback(&mission_event_callback);

  global_wp_ptr_->print_return_code("Subscribe mission event result: ", ret);

  return ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
WaypointFlyingModule::subscribe_waypoint_v2_state_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::SubscribeWaypointV2State::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2State::Response>
        res)
{
  (void)req;

  res->result = subscribe_waypoint_v2_mission_state();

  if (res->result)
  {
    RCLCPP_INFO(get_logger(),
                "Successfully registered waypoint mission state callback");
  }
  else
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to register waypoint mission state callback");
  }
}

void
WaypointFlyingModule::pause_waypoint_v2_mission_callback(
    const std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Response> res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "pause_waypoint_v2_mission_callback");

  res->result = true;
}

void
WaypointFlyingModule::resume_waypoint_v2_mission_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::ResumeWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::ResumeWaypointV2Mission::Response>
        res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "resume_waypoint_v2_mission_callback");

  res->result = true;
}

void
WaypointFlyingModule::print_return_code(const std::string &text,
                                        T_DjiReturnCode code) const
{
  if (const auto *err = psdk_utils::findError(code))
  {
    if (err->suggestion)
    {
      RCLCPP_INFO(get_logger(), "%s%s (%s)", text.c_str(), err->description,
                  err->suggestion);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "%s%s", text.c_str(), err->description);
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "%s0x%lx", text.c_str(),
                static_cast<unsigned long>(code));
  }
}

void
WaypointFlyingModule::start_waypoint_v2_mission_callback(
    const std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Response> res)
    const
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  // "start_waypoint_v2_mission_callback");

  res->result = true;
  T_DjiReturnCode ret = DjiWaypointV2_Start();

  print_return_code("Start mission result: ", ret);

  if (ret > 0)
  {
    res->result = false;
  }
}

void
WaypointFlyingModule::stop_waypoint_v2_mission_callback(
    const std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Response> res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "stop_waypoint_v2_mission_callback");

  res->result = true;
}

void
WaypointFlyingModule::download_waypoint_v2_mission_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::DownloadWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::DownloadWaypointV2Mission::Response>
        res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "download_waypoint_v2_mission_callback");

  res->result = true;
}

void
WaypointFlyingModule::upload_waypoint_v2_mission_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::UploadWaypointV2Mission::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Mission::Response>
        res) const
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  // "upload_waypoint_v2_mission_callback");

  T_DjiReturnCode ret = DjiWaypointV2_UploadMission(&ms_);
  print_return_code("Upload mission result: ", ret);

  res->result = true;

  if (ret > 0)
  {
    res->result = false;
  }
}

void
WaypointFlyingModule::generate_waypoint_v2_action_callback(
    const std::shared_ptr<
        psdk_interfaces::srv::GenerateWaypointV2Action::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::GenerateWaypointV2Action::Response>
        res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "generate_waypoint_v2_action_callback");
  res->result = true;
}

void
WaypointFlyingModule::init_waypoint_v2_setting_callback(
    const std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Request>
        request,
    std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Response>
        response)
{
  response->result =
      init_waypoint_v2_setting(request->waypoint_v2_init_settings);

  print_mission_summary();

}

double
WaypointFlyingModule::waypoint_distance(
    const T_DjiWaypointV2 &a,
    const T_DjiWaypointV2 &b) const
{
  constexpr double kEarthRadius = 6378137.0;

  const double dlat = b.latitude - a.latitude;
  const double dlon = b.longitude - a.longitude;

  const double h =
      std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
      std::cos(a.latitude) * std::cos(b.latitude) *
          std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

  return 2.0 * kEarthRadius *
         std::atan2(std::sqrt(h), std::sqrt(1.0 - h));
}

double
WaypointFlyingModule::waypoint_bearing(
    const T_DjiWaypointV2 &a,
    const T_DjiWaypointV2 &b) const
{
  const double dlon = b.longitude - a.longitude;

  const double x =
      std::sin(dlon) * std::cos(b.latitude);

  const double y =
      std::cos(a.latitude) * std::sin(b.latitude) -
      std::sin(a.latitude) * std::cos(b.latitude) *
          std::cos(dlon);

  double heading = std::atan2(x, y) * 180.0 / M_PI;

  if (heading < 0.0)
    heading += 360.0;

  return heading;
}

double
WaypointFlyingModule::waypoint_turn_angle(
    const T_DjiWaypointV2 &a,
    const T_DjiWaypointV2 &b,
    const T_DjiWaypointV2 &c) const
{
  const double h1 = waypoint_bearing(a, b);
  const double h2 = waypoint_bearing(b, c);

  double angle = h2 - h1;

  while (angle > 180.0)
    angle -= 360.0;

  while (angle < -180.0)
    angle += 360.0;

  return angle;
}
void
WaypointFlyingModule::print_mission_summary() const
{
  constexpr double kRadToDeg = 180.0 / M_PI;

  if (ms_.missTotalLen == 0)
  {
    RCLCPP_INFO(get_logger(), "Mission is empty.");
    return;
  }

  RCLCPP_INFO(
      get_logger(),
      "Mission %u: %u waypoint(s), repeat=%u, max_speed=%.1f m/s, auto_speed=%.1f m/s",
      ms_.missionID,
      ms_.missTotalLen,
      ms_.repeatTimes,
      ms_.maxFlightSpeed,
      ms_.autoFlightSpeed);

  double total_distance = 0.0;

  for (uint16_t i = 0; i < ms_.missTotalLen; ++i)
  {
    const auto &wp = mission_[i];

    double segment_distance = 0.0;
    double heading = 0.0;
    double turn = 0.0;

    if (i > 0)
    {
      segment_distance =
          waypoint_distance(
              mission_[i - 1],
              mission_[i]);

      heading =
          waypoint_bearing(
              mission_[i - 1],
              mission_[i]);

      total_distance += segment_distance;
    }

    if (i > 1)
    {
      turn =
          waypoint_turn_angle(
              mission_[i - 2],
              mission_[i - 1],
              mission_[i]);
    }

    RCLCPP_INFO(
        get_logger(),
        "WP%-2u "
        "lat=%10.7f° "
        "lon=%11.7f° "
        "h=%5.1fm "
        "dist=%6.2fm "
        "hdg=%6.1f° "
        "turn=%6.1f° "
        "damp=%4ucm "
        "type=%s",
        i,
        wp.latitude * kRadToDeg,
        wp.longitude * kRadToDeg,
        wp.relativeHeight,
        segment_distance,
        heading,
        turn,
        wp.dampingDistance,
        flight_path_mode_to_string(wp.waypointType).c_str());

    if (i == 0 &&
        wp.waypointType ==
            DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN)
    {
      RCLCPP_WARN(
          get_logger(),
          "  First waypoint is a coordinated turn.");
    }

    if (i > 0)
    {
      if (wp.dampingDistance >
          segment_distance * 50.0)
      {
        RCLCPP_WARN(
            get_logger(),
            "  Damping (%.2fm) exceeds half the segment length (%.2fm).",
            wp.dampingDistance / 100.0,
            segment_distance / 2.0);
      }

      if (wp.waypointType ==
              DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN &&
          segment_distance < 3.0)
      {
        RCLCPP_WARN(
            get_logger(),
            "  Coordinated turn segment is only %.2fm.",
            segment_distance);
      }
    }
  }

  RCLCPP_INFO(
      get_logger(),
      "Total mission length: %.2f m",
      total_distance);
}

bool
WaypointFlyingModule::init_waypoint_v2_setting(
    const psdk_interfaces::msg::WaypointV2InitSetting &settings)
{
  RCLCPP_INFO(get_logger(), "Initializing Waypoint V2 mission");

  //
  // Currently unused
  //
  // uint16_t polygon_num = settings.polygon_num;
  // float radius = settings.radius;
  // uint16_t action_num = settings.action_num;

  mission_.resize(settings.mission.size());

  ms_ = {};
  ms_.missionID = std::rand();
  ms_.repeatTimes = settings.repeat_times;
  ms_.finishedAction = static_cast<E_DJIWaypointV2MissionFinishedAction>(
      settings.finished_action);

  ms_.maxFlightSpeed = settings.max_flight_speed;
  ms_.autoFlightSpeed = settings.auto_flight_speed;
  ms_.actionWhenRcLost =
      DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;

  ms_.gotoFirstWaypointMode =
      static_cast<E_DJIWaypointV2MissionGotoFirstWaypointMode>(
          settings.goto_first_waypoint_mode);

  ms_.missTotalLen = mission_.size();
  for (size_t i = 0; i < mission_.size(); ++i)
  {
    fill_waypoint(settings.mission[i], mission_[i]);
  }
  action_list_ = {};
  action_list_.actions = nullptr;
  action_list_.actionNum = 0;

  ms_.mission = mission_.data();
  ms_.actionList = action_list_;

  return true;
}

std::string
WaypointFlyingModule::flight_path_mode_to_string(
    E_DJIWaypointV2FlightPathMode mode) const
{
  switch (mode)
  {
    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_CURVE:
      return "Curve Fly-By";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_CURVE_AND_STOP:
      return "Curve Stop";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP:
      return "Straight Stop";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN:
      return "Coord Turn";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_FIRST_POINT_ALONG_STRAIGHT_LINE:
      return "First Straight";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_STRAIGHT_OUT:
      return "Straight Out";

    case DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_UNKNOWN:
      return "Unknown";

    default:
      return "Invalid";
  }
}

void
WaypointFlyingModule::fill_waypoint(const psdk_interfaces::msg::WaypointV2 &src,
                                    T_DjiWaypointV2 &dst)
{
  dst = {};

  dst.longitude = src.longitude;
  dst.latitude = src.latitude;

  dst.relativeHeight = src.relative_height;

  dst.waypointType =
      static_cast<E_DJIWaypointV2FlightPathMode>(src.waypoint_type);

  dst.headingMode = static_cast<E_DJIWaypointV2HeadingMode>(src.heading_mode);

  dst.config.useLocalCruiseVel = src.config.use_local_cruise_vel;
  dst.config.useLocalMaxVel = src.config.use_local_max_vel;
  dst.dampingDistance = src.damping_distance;
  dst.heading = src.heading;
  dst.turnMode = static_cast<E_DJIWaypointV2TurnMode>(src.turn_mode);
  dst.maxFlightSpeed = src.max_flight_speed;
  dst.autoFlightSpeed = src.auto_flight_speed;

  //
  // Only used with HEADING_TOWARDS_POINT_OF_INTEREST.
  // Not exposed yet.
  //
  /*
  dst.pointOfInterest.positionX = src.position_x;
  dst.pointOfInterest.positionY = src.position_y;
  dst.pointOfInterest.positionZ = src.position_z;
  */
  if (src.heading_mode ==
        DJI_WAYPOINT_V2_HEADING_TOWARDS_POINT_OF_INTEREST)
  {
    RCLCPP_WARN(
        get_logger(),
        "Point-of-interest heading mode selected, "
        "but pointOfInterest is not yet supported.");
  }
}

#if 0
void
WaypointFlyingModule::init_waypoint_v2_setting_callback(
    const std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Request>
        request,
    std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Response>
        response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "init_waypoint_v2_setting_callback");

  uint16_t polygonNum = request->polygon_num;
  float radius = request->radius;

  uint16_t actionNum = request->action_num;
  srand(int(time(0)));

  ms = new T_DjiWayPointV2MissionSettings();

  ms->missionID = rand();  // uint32_t
  ms->repeatTimes = request->waypoint_v2_init_settings
                        .repeat_times;  // No repeat, 1 = execute to times
  /// ms->finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
  ms->finishedAction = static_cast<E_DJIWaypointV2MissionFinishedAction>(
      request->waypoint_v2_init_settings.finished_action);
  ms->maxFlightSpeed = request->waypoint_v2_init_settings.max_flight_speed;
  ms->autoFlightSpeed = request->waypoint_v2_init_settings.auto_flight_speed;
  ms->actionWhenRcLost =
      DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
  // ms->gotoFirstWaypointMode =
  // DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
  // DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT
  ms->gotoFirstWaypointMode =
      static_cast<E_DJIWaypointV2MissionGotoFirstWaypointMode>(
          request->waypoint_v2_init_settings.goto_first_waypoint_mode);
  ms->missTotalLen = request->waypoint_v2_init_settings.mission.size();

  ms->mission =
      (T_DjiWaypointV2 *)malloc(ms->missTotalLen * sizeof(T_DjiWaypointV2));

  for (uint16_t i = 0; i < request->waypoint_v2_init_settings.mission.size();
       i++)
  {
    // T_DjiWaypointV2 wp;
    T_DjiWaypointV2 wp{};

    wp.longitude = request->waypoint_v2_init_settings.mission[i].longitude;
    wp.latitude = request->waypoint_v2_init_settings.mission[i].latitude;
    wp.relativeHeight =
        request->waypoint_v2_init_settings.mission[i].relative_height;
    wp.waypointType = static_cast<E_DJIWaypointV2FlightPathMode>(
        request->waypoint_v2_init_settings.mission[i].waypoint_type);
    /// wp.waypointType =
    /// DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
    wp.headingMode = static_cast<E_DJIWaypointV2HeadingMode>(
        request->waypoint_v2_init_settings.mission[i].heading_mode);
    /// wp.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
    wp.config.useLocalCruiseVel = request->waypoint_v2_init_settings.mission[i]
                                      .config.use_local_cruise_vel;
    wp.config.useLocalMaxVel =
        request->waypoint_v2_init_settings.mission[i].config.use_local_max_vel;
    wp.dampingDistance =
        request->waypoint_v2_init_settings.mission[i].damping_distance;
    wp.heading = request->waypoint_v2_init_settings.mission[i].heading;
    wp.turnMode = static_cast<E_DJIWaypointV2TurnMode>(
        request->waypoint_v2_init_settings.mission[i].turn_mode);
    /// wp.turnMode = DJI_WAYPOINT_V2_TURN_MODE_UNKNOWN;
    wp.maxFlightSpeed =
        request->waypoint_v2_init_settings.mission[i].max_flight_speed;
    wp.autoFlightSpeed =
        request->waypoint_v2_init_settings.mission[i].auto_flight_speed;
    ms->mission[i] = wp;
  }

  /// ms->mission =  // T_DjiWaypointV2 *mission;

  T_DJIWaypointV2ActionList alist;
  alist.actions = 0;
  alist.actionNum = 0;
  ms->actionList = alist;

#if 0
  waypointV2Vector.pointOfInterest.positionX = request->waypoint_v2_init_settings.mission[i].position_x;
  waypointV2Vector.pointOfInterest.positionY = request->waypoint_v2_init_settings.mission[i].position_y;
  waypointV2Vector.pointOfInterest.positionZ = request->waypoint_v2_init_settings.mission[i].position_z;
#endif

  //  T_DjiWaypointV2GlobalCruiseSpeed cruise_speed = 5.0;
  //  T_DjiReturnCode speedres =
  //  DjiWaypointV2_SetGlobalCruiseSpeed(cruise_speed); std::cerr << "speedres:
  //  " << speedres << std::endl;

  response->result = true;
  // if (speedres > 0) {
  //     response->result = false;
  //   }
}
#endif
void
WaypointFlyingModule::upload_waypoint_v2_action_callback(
    const std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Request>
        req,
    std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Response> res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "upload_waypoint_v2_action_callback");
  res->result = true;
}

}  // namespace psdk_ros2
