/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file flight_control.cpp
 *
 * @brief Flight control module implementation. This module is responsible for
 * handling the flight control commands and implementating the basic flight
 * behaviors such as takeoff, landing, return to home. Moreover, it enables and
 * disables the obstacle avoidance functionality.
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "psdk_wrapper/modules/flight_control.hpp"

#include <std_msgs/msg/detail/string__struct.hpp>

namespace psdk_ros2
{

FlightControlModule::FlightControlModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating FlightControlModule");
}

FlightControlModule::~FlightControlModule()
{
  RCLCPP_INFO(get_logger(), "Destroying FlightControlModule");
}

FlightControlModule::CallbackReturn
FlightControlModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring FlightControlModule");

  flight_control_generic_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "psdk_ros2/flight_control_setpoint_generic", 10,
      std::bind(&FlightControlModule::flight_control_generic_cb, this,
                std::placeholders::_1));
  flight_control_position_yaw_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "psdk_ros2/flight_control_setpoint_ENUposition_yaw", 10,
      std::bind(&FlightControlModule::flight_control_position_yaw_cb, this,
                std::placeholders::_1));
  flight_control_velocity_yawrate_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate", 10,
          std::bind(&FlightControlModule::flight_control_velocity_yawrate_cb,
                    this, std::placeholders::_1));
  flight_control_body_velocity_yawrate_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate", 10,
          std::bind(
              &FlightControlModule::flight_control_body_velocity_yawrate_cb,
              this, std::placeholders::_1));
  flight_control_rollpitch_yawrate_thrust_sub_ =
      create_subscription<sensor_msgs::msg::Joy>(
          "psdk_ros2/flight_control_setpoint_rollpitch_yawrate_thrust", 10,
          std::bind(
              &FlightControlModule::flight_control_rollpitch_yawrate_thrust_cb,
              this, std::placeholders::_1));

  // ROS 2 Services
  set_home_from_gps_srv_ = create_service<SetHomeFromGPS>(
      "psdk_ros2/set_home_from_gps",
      std::bind(&FlightControlModule::set_home_from_gps_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_home_from_current_location_srv_ = create_service<Trigger>(
      "psdk_ros2/set_home_from_current_location",
      std::bind(&FlightControlModule::set_home_from_current_location_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_go_home_altitude_srv_ = create_service<SetGoHomeAltitude>(
      "psdk_ros2/set_go_home_altitude",
      std::bind(&FlightControlModule::set_go_home_altitude_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  get_go_home_altitude_srv_ = create_service<GetGoHomeAltitude>(
      "psdk_ros2/get_go_home_altitude",
      std::bind(&FlightControlModule::get_go_home_altitude_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_go_home_srv_ = create_service<Trigger>(
      "psdk_ros2/start_go_home",
      std::bind(&FlightControlModule::start_go_home_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  cancel_go_home_srv_ = create_service<Trigger>(
      "psdk_ros2/cancel_go_home",
      std::bind(&FlightControlModule::cancel_go_home_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  obtain_ctrl_authority_srv_ = create_service<Trigger>(
      "psdk_ros2/obtain_ctrl_authority",
      std::bind(&FlightControlModule::obtain_ctrl_authority_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  release_ctrl_authority_srv_ = create_service<Trigger>(
      "psdk_ros2/release_ctrl_authority",
      std::bind(&FlightControlModule::release_ctrl_authority_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  turn_on_motors_srv_ = create_service<Trigger>(
      "psdk_ros2/turn_on_motors",
      std::bind(&FlightControlModule::turn_on_motors_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  turn_off_motors_srv_ = create_service<Trigger>(
      "psdk_ros2/turn_off_motors",
      std::bind(&FlightControlModule::turn_off_motors_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  takeoff_srv_ = create_service<Trigger>(
      "psdk_ros2/takeoff",
      std::bind(&FlightControlModule::start_takeoff_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  land_srv_ = create_service<Trigger>(
      "psdk_ros2/land",
      std::bind(&FlightControlModule::start_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  cancel_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/cancel_landing",
      std::bind(&FlightControlModule::cancel_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_confirm_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/start_confirm_landing",
      std::bind(&FlightControlModule::start_confirm_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_force_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/start_force_landing",
      std::bind(&FlightControlModule::start_force_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_horizontal_vo_obstacle_avoidance",
          std::bind(
              &FlightControlModule::set_horizontal_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_horizontal_radar_obstacle_avoidance",
          std::bind(
              &FlightControlModule::set_horizontal_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_upwards_vo_obstacle_avoidance_srv_ = create_service<SetObstacleAvoidance>(
      "psdk_ros2/set_upwards_vo_obstacle_avoidance",
      std::bind(&FlightControlModule::set_upwards_vo_obstacle_avoidance_cb,
                this, std::placeholders::_1, std::placeholders::_2));
  set_upwards_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_upwards_radar_obstacle_avoidance",
          std::bind(
              &FlightControlModule::set_upwards_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_downwards_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_downwards_vo_obstacle_avoidance",
          std::bind(
              &FlightControlModule::set_downwards_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_horizontal_vo_obstacle_avoidance",
          std::bind(
              &FlightControlModule::get_horizontal_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_upwards_vo_obstacle_avoidance_srv_ = create_service<GetObstacleAvoidance>(
      "psdk_ros2/get_upwards_vo_obstacle_avoidance",
      std::bind(&FlightControlModule::get_upwards_vo_obstacle_avoidance_cb,
                this, std::placeholders::_1, std::placeholders::_2));
  get_upwards_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_upwards_radar_obstacle_avoidance",
          std::bind(
              &FlightControlModule::get_upwards_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_downwards_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_downwards_vo_obstacle_avoidance",
          std::bind(
              &FlightControlModule::get_downwards_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_horizontal_radar_obstacle_avoidance",
          std::bind(
              &FlightControlModule::get_horizontal_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

FlightControlModule::CallbackReturn
FlightControlModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating FlightControlModule");
  return CallbackReturn::SUCCESS;
}

FlightControlModule::CallbackReturn
FlightControlModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating FlightControlModule");
  return CallbackReturn::SUCCESS;
}

FlightControlModule::CallbackReturn
FlightControlModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up FlightControlModule");
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
  return CallbackReturn::SUCCESS;
}

FlightControlModule::CallbackReturn
FlightControlModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Shutting down FlightControlModule");
  return CallbackReturn::SUCCESS;
}

bool
FlightControlModule::init(
    const sensor_msgs::msg::NavSatFix &current_gps_position)
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Flight control module is already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating flight control module");
  T_DjiFlightControllerRidInfo rid_info;
  rid_info.latitude = psdk_utils::deg_to_rad(current_gps_position.latitude);
  rid_info.longitude = psdk_utils::deg_to_rad(current_gps_position.longitude);
  rid_info.altitude = current_gps_position.altitude;

  T_DjiReturnCode return_code = DjiFlightController_Init(rid_info);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not initialize flight control module. Error code is: %ld",
        return_code);
    return false;
  }
  is_module_initialized_ = true;
  joy_command_time_ = get_clock()->now();
  return true;
}

bool
FlightControlModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing flight control module");
  T_DjiReturnCode return_code = DjiFlightController_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not deinitialize the flight control module. Error code: %ld",
        return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

const std::string &FlightControlModule::flight_control_mode_to_string(const FlightControlMode mode)
{
  static const std::string unknown = "Unknown";

  auto it = flight_control_mode_str.find(mode);
  if (it != flight_control_mode_str.end())
  {
    return it->second;
  }

  return unknown;
}

const std::string &FlightControlModule::flight_control_joystick_mode_to_string(
    const FlightControlJoystickMode mode)
{
  static const std::string unknown = "Unknown";

  auto it = flight_control_joystick_mode_str.find(mode);
  if (it != flight_control_joystick_mode_str.end())
  {
    return it->second;
  }

  return unknown;
}


std::string FlightControlModule::get_flight_control_mode_status_str() const
{
  float time_since_joy_command = (get_clock()->now() - joy_command_time_).seconds();

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "{"
      << "\"flight_mode\":\"" << fc_mode << "\","
      << "\"flight_mode_str\":\"" << flight_control_mode_to_string(fc_mode) << "\","
      << "\"joy_mode\":" << fc_joystick_mode << ","
      << "\"joy_mode_str\":" << flight_control_joystick_mode_to_string(fc_joystick_mode) << ","
      << "\"time_since_joy_command\":\"" << time_since_joy_command << "\","
      << "}";

  return oss.str();
}

bool
FlightControlModule::execute_flight_command(FlightControlCommand command)
{
  switch (command)
  {
    case FLIGHT_CONTROL_CMD_NONE:
      return true;

    case FLIGHT_CONTROL_CMD_TAKEOFF:
      return start_takeoff();

    case FLIGHT_CONTROL_CMD_LAND:
      return start_landing();

    case FLIGHT_CONTROL_CMD_CONFIRM_LAND:
      return start_confirm_landing();

    case FLIGHT_CONTROL_CMD_FORCE_LAND:
      return start_force_landing();

    case FLIGHT_CONTROL_CMD_CANCEL_LAND:
      return cancel_landing();

    case FLIGHT_CONTROL_CMD_GO_HOME:
      return start_go_home();

    case FLIGHT_CONTROL_CMD_CANCEL_GO_HOME:
      return cancel_go_home();

    case FLIGHT_CONTROL_CMD_OBTAIN_JOYSTICK:
      return obtain_ctrl_authority();

    case FLIGHT_CONTROL_CMD_RELEASE_JOYSTICK:
      return release_ctrl_authority();

    default:
      RCLCPP_ERROR(get_logger(), "Unknown flight control command %d",
                   static_cast<int>(command));
      return false;
  }
}

void
FlightControlModule::update_flight_control_mode(const uint8_t &flight_status)
{
  auto ret = fc_mode;

  switch (fc_command)
  {
    case FLIGHT_CONTROL_CMD_TAKEOFF:  // takeoff
      if (flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR)
      {
        fc_mode = FLIGHT_CONTROL_MODE_NONE;
      }
      break;

    case FLIGHT_CONTROL_CMD_CANCEL_LAND:
    case FLIGHT_CONTROL_CMD_CANCEL_GO_HOME:
      fc_mode = FLIGHT_CONTROL_MODE_NONE;
      fc_command = FLIGHT_CONTROL_CMD_NONE;

    case FLIGHT_CONTROL_CMD_LAND:
    case FLIGHT_CONTROL_CMD_FORCE_LAND:
    case FLIGHT_CONTROL_CMD_CONFIRM_LAND:
    case FLIGHT_CONTROL_CMD_GO_HOME:
      if (flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED)
      {
        fc_mode = FLIGHT_CONTROL_MODE_NONE;
        fc_command = FLIGHT_CONTROL_CMD_NONE;
      }
      break;
    default:;
  }
}

void
FlightControlModule::set_home_from_gps_cb(
    const std::shared_ptr<SetHomeFromGPS::Request> request,
    const std::shared_ptr<SetHomeFromGPS::Response> response)
{
  T_DjiFlightControllerHomeLocation home_location;
  home_location.latitude = psdk_utils::deg_to_rad(request->latitude);
  home_location.longitude = psdk_utils::deg_to_rad(request->longitude);
  auto result =
      DjiFlightController_SetHomeLocationUsingGPSCoordinates(home_location);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home location using the given gps "
                 "coordinates. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Home position set to coordinates lat: %f, long: %f",
              request->latitude, request->longitude);
  response->success = true;
}
void
FlightControlModule::set_home_from_current_location_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result =
      DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set the home location using current aicraft position. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Home location has been set to current position!");
  response->success = true;
}

void
FlightControlModule::set_go_home_altitude_cb(
    const std::shared_ptr<SetGoHomeAltitude::Request> request,
    const std::shared_ptr<SetGoHomeAltitude::Response> response)
{
  E_DjiFlightControllerGoHomeAltitude home_altitude = request->altitude;
  auto result = DjiFlightController_SetGoHomeAltitude(home_altitude);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home altitude at the current aicraft "
                 "location. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Home altitude has been set to: %d",
              request->altitude);
  response->success = true;
}

void
FlightControlModule::get_go_home_altitude_cb(
    const std::shared_ptr<GetGoHomeAltitude::Request> request,
    const std::shared_ptr<GetGoHomeAltitude::Response> response)
{
  (void)request;
  auto result =
      DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not get the home location using current aicraft location. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  response->success = true;
}
bool
FlightControlModule::start_go_home()
{
  auto result = DjiFlightController_StartGoHome();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not start go to home action. Error code: %ld", result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_GO_HOME;
  fc_command = FLIGHT_CONTROL_CMD_GO_HOME;
  RCLCPP_INFO(get_logger(), "Go Home action started");
  return true;
}

void
FlightControlModule::start_go_home_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = start_go_home();
}

bool
FlightControlModule::cancel_go_home()
{
  auto result = DjiFlightController_CancelGoHome();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not cancel go to home action. Error code: %ld", result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_NONE;
  fc_command = FLIGHT_CONTROL_CMD_CANCEL_GO_HOME;

  RCLCPP_INFO(get_logger(), "Go Home action has been cancelled");
  return true;
}

void
FlightControlModule::cancel_go_home_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = cancel_go_home();
}

bool
FlightControlModule::obtain_ctrl_authority()
{
  fc_command = FLIGHT_CONTROL_CMD_OBTAIN_JOYSTICK;

  auto result = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not obtain control authority. Error code is: %ld",
                 result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_JOYSTICK;
  RCLCPP_INFO(get_logger(), "Control authority obtained");
  return true;
}

void
FlightControlModule::obtain_ctrl_authority_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = obtain_ctrl_authority();
}

bool
FlightControlModule::release_ctrl_authority()
{
  fc_command = FLIGHT_CONTROL_CMD_RELEASE_JOYSTICK;

  auto result = DjiFlightController_ReleaseJoystickCtrlAuthority();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not release control authority. Error code is: %ld",
                 result);
    return false;
  }

  fc_mode = FLIGHT_CONTROL_MODE_NONE;
  fc_joystick_mode = FLIGHT_CONTROL_MODE_JOY_NONE;

  RCLCPP_INFO(get_logger(), "Control authority released");
  return true;
}

void
FlightControlModule::release_ctrl_authority_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = release_ctrl_authority();
}
void
FlightControlModule::set_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  auto result =
      DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
          status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal vo obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Horizontal VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
FlightControlModule::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  auto result =
      DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
          status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set horizontal radar obstacle avoidance status. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Horizontal Radar obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
FlightControlModule::set_downwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  auto result =
      DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
          status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set downwards visual obstacle avoidance status. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Downwards VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
FlightControlModule::set_upwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  auto result =
      DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set upwards visual obstacle avoidance status. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Upwards VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
FlightControlModule::set_upwards_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  auto result =
      DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set upwards radar obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Upwards Radar obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
FlightControlModule::get_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  auto result =
      DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
          &status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get horizontal vo obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
FlightControlModule::get_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  auto result =
      DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
          &status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not get horizontal radar obstacle avoidance status. Error "
        "code is: %ld",
        result);
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
FlightControlModule::get_downwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  auto result =
      DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
          &status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get downwards vo obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
FlightControlModule::get_upwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  auto result =
      DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
          &status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get upwards vo obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
FlightControlModule::get_upwards_radar_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  auto result =
      DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(&status);
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get upwards radar obstacle avoidance status. Error "
                 "code is: %ld",
                 result);
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
FlightControlModule::turn_on_motors_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_TurnOnMotors();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get turn ON motors. Error code is: %ld", result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Motors have been turned ON");
  response->success = true;
}

void
FlightControlModule::turn_off_motors_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_TurnOffMotors();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get turn OFF motors. Error code is: %ld", result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Motors have been turned OFF");
  response->success = true;
}

bool
FlightControlModule::start_takeoff()
{
  auto result = DjiFlightController_StartTakeoff();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start takeoff! Error code is: %ld",
                 result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_TAKEOFF;
  fc_command = FLIGHT_CONTROL_CMD_TAKEOFF;
  RCLCPP_INFO(get_logger(), "Starting Take Off");
  return true;
}

void
FlightControlModule::start_takeoff_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = start_takeoff();
}

bool
FlightControlModule::start_landing()
{
  auto result = DjiFlightController_StartLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start landing! Error code is: %ld",
                 result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_LAND;
  fc_command = FLIGHT_CONTROL_CMD_LAND;

  RCLCPP_INFO(get_logger(), "Starting Landing");
  return true;
}

void
FlightControlModule::start_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = start_landing();
}

bool
FlightControlModule::cancel_landing()
{
  fc_command = FLIGHT_CONTROL_CMD_CANCEL_LAND;
  auto result = DjiFlightController_CancelLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not cancel landing! Error code is: %ld",
                 result);
    return false;
  }
  fc_mode = FLIGHT_CONTROL_MODE_NONE;

  RCLCPP_INFO(get_logger(), "Landing has been cancelled");
  return true;
}

void
FlightControlModule::cancel_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = cancel_landing();
}
bool
FlightControlModule::start_confirm_landing()
{
  fc_command = FLIGHT_CONTROL_CMD_CONFIRM_LAND;

  auto result = DjiFlightController_StartConfirmLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not confirm landing! Error code is: %ld",
                 result);
    return false;
  }

  fc_mode = FLIGHT_CONTROL_MODE_CONFIRM_LAND;

  RCLCPP_INFO(get_logger(), "Landing has been confirmed");
  return true;
}

void
FlightControlModule::start_confirm_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = start_confirm_landing();
}

bool
FlightControlModule::start_force_landing()
{
  fc_command = FLIGHT_CONTROL_CMD_FORCE_LAND;

  auto result = DjiFlightController_StartForceLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not force landing! Error code is: %ld",
                 result);
    return false;
  }

  fc_mode = FLIGHT_CONTROL_MODE_FORCE_LAND;

  RCLCPP_INFO(get_logger(), "Force Landing!");
  return true;
}

void
FlightControlModule::start_force_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  response->success = start_force_landing();
}
void
FlightControlModule::flight_control_generic_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  /** @todo implemnent generic control functionality */
  (void)msg;
  RCLCPP_WARN(get_logger(),
              "Generic control setpoint is not currently implemented!");
}

void
FlightControlModule::flight_control_position_yaw_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  fc_joystick_mode = FLIGHT_CONTROL_MODE_JOY_POSITION_YAW;
  joy_command_time_ = get_clock()->now();
  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd;
  float yaw_cmd;

  /* Note: The position input expected by DJI is ground fixed frame NEU,
   * Following REP 103, the x and y setpoints are inverted.
   */
  x_cmd = y_setpoint;
  y_cmd = x_setpoint;
  z_cmd = z_setpoint;

  /* Note: The input yaw is assumed to be following REP 103, thus a yaw rotation
   wrt to ENU frame. DJI uses rotation with respect to NED.Thus, the
   rotation needs to be transformed before sending it to the FCU
   */
  tf2::Matrix3x3 rotation_FLU2ENU;
  rotation_FLU2ENU.setRPY(0.0, 0.0, yaw_setpoint);
  tf2::Matrix3x3 rotation_FRD2NED(psdk_utils::R_NED2ENU.transpose() *
                                  rotation_FLU2ENU *
                                  psdk_utils::R_FLU2FRD.transpose());
  double temp1, temp2, temp_yaw;
  rotation_FRD2NED.getRPY(temp1, temp2, temp_yaw);
  yaw_cmd = static_cast<float>(temp_yaw);
  yaw_cmd = psdk_utils::rad_to_deg(yaw_cmd);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
FlightControlModule::flight_control_velocity_yawrate_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  fc_joystick_mode = FLIGHT_CONTROL_MODE_JOY_VELOCITY_YAWRATE;
  joy_command_time_ = get_clock()->now();

  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  /**
   * Note: DJI is expecting velocity commands with a ground-fixed frame NEU.
   * Input is converted from ENU to NEU.
   */
  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  x_cmd = y_setpoint;
  y_cmd = x_setpoint;
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
FlightControlModule::flight_control_body_velocity_yawrate_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  fc_joystick_mode = FLIGHT_CONTROL_MODE_JOY_BODY_VELOCITY_YAWRATE;
  joy_command_time_ = get_clock()->now();

  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  // The Horizontal body coordinate frame of DJI is defined as FRU. Here the
  // input is assumed to be in FLU, transform from FL to FR.
  x_cmd = x_setpoint;
  y_cmd = -y_setpoint;
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
FlightControlModule::flight_control_rollpitch_yawrate_thrust_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  fc_joystick_mode = FLIGHT_CONTROL_MODE_JOY_ROLL_PITCH_YAWRATE_THRUST;
  joy_command_time_ = get_clock()->now();

  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  // Transform from FL to FR
  x_cmd = psdk_utils::rad_to_deg(x_setpoint);
  y_cmd = psdk_utils::rad_to_deg(-y_setpoint);

  /**
   * Note: Thrust input is expected here. Range 0 - 100 %.
   */
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

}  // namespace psdk_ros2
