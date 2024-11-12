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

#include "dji_waypoint_v2.h"
#include "psdk_wrapper/modules/waypoint_flying.hpp"
#include <unistd.h>
#include <functional>

psdk_ros2::WaypointFlyingModule * wfm_pointer;

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

  state_push_publisher = this->create_publisher<psdk_interfaces::msg::WaypointV2MissionStatePush>("psdk_ros2/waypointV2_mission_state", 10);
  event_push_publisher = this->create_publisher<psdk_interfaces::msg::WaypointV2MissionEventPush>("psdk_ros2/waypointV2_mission_event", 10);

  subscribe_waypoint_v2_event_service = this->create_service<psdk_interfaces::srv::SubscribeWaypointV2Event>("psdk_ros2/waypointV2_subscribeMissionEvent", std::bind(&WaypointFlyingModule::subscribe_waypoint_v2_event_callback, this, std::placeholders::_1, std::placeholders::_2));
  subscribe_waypoint_v2_state_service = this->create_service<psdk_interfaces::srv::SubscribeWaypointV2State>("psdk_ros2/waypointV2_subscribeMissionState", std::bind(&WaypointFlyingModule::subscribe_waypoint_v2_state_callback, this, std::placeholders::_1, std::placeholders::_2));
  pause_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::PauseWaypointV2Mission>("psdk_ros2/waypointV2_pauseMission", std::bind(&WaypointFlyingModule::pause_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  resume_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::ResumeWaypointV2Mission>("psdk_ros2/waypointV2_resumeMission", std::bind(&WaypointFlyingModule::resume_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  stop_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::StopWaypointV2Mission>("psdk_ros2/waypointV2_stopMission", std::bind(&WaypointFlyingModule::stop_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  start_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::StartWaypointV2Mission>("psdk_ros2/waypointV2_startMission", std::bind(&WaypointFlyingModule::start_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  download_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::DownloadWaypointV2Mission>("psdk_ros2/waypointV2_downloadMission", std::bind(&WaypointFlyingModule::download_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  upload_waypoint_v2_mission_service = this->create_service<psdk_interfaces::srv::UploadWaypointV2Mission>("psdk_ros2/waypointV2_uploadMission", std::bind(&WaypointFlyingModule::upload_waypoint_v2_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
  generate_waypoint_v2_action_service = this->create_service<psdk_interfaces::srv::GenerateWaypointV2Action>("psdk_ros2/waypointV2_generateActions", std::bind(&WaypointFlyingModule::generate_waypoint_v2_action_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  init_waypoint_v2_setting_service = this->create_service<psdk_interfaces::srv::InitWaypointV2Setting>("psdk_ros2/waypointV2_initSetting", std::bind(&WaypointFlyingModule::init_waypoint_v2_setting_callback, this, std::placeholders::_1, std::placeholders::_2));
  upload_waypoint_v2_action_service = this->create_service<psdk_interfaces::srv::UploadWaypointV2Action>("psdk_ros2/waypointV2_uploadActions", std::bind(&WaypointFlyingModule::upload_waypoint_v2_action_callback, this, std::placeholders::_1, std::placeholders::_2));
  
#if 0  
  flight_control_generic_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "psdk_ros2/flight_control_setpoint_generic", 10,
      std::bind(&WaypointFlyingModule::flight_control_generic_cb, this,
                std::placeholders::_1));

  // ROS 2 Services
  set_home_from_gps_srv_ = create_service<SetHomeFromGPS>(
      "psdk_ros2/set_home_from_gps",
      std::bind(&WaypointFlyingModule::set_home_from_gps_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_home_from_current_location_srv_ = create_service<Trigger>(
      "psdk_ros2/set_home_from_current_location",
      std::bind(&WaypointFlyingModule::set_home_from_current_location_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_go_home_altitude_srv_ = create_service<SetGoHomeAltitude>(
      "psdk_ros2/set_go_home_altitude",
      std::bind(&WaypointFlyingModule::set_go_home_altitude_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  get_go_home_altitude_srv_ = create_service<GetGoHomeAltitude>(
      "psdk_ros2/get_go_home_altitude",
      std::bind(&WaypointFlyingModule::get_go_home_altitude_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_go_home_srv_ = create_service<Trigger>(
      "psdk_ros2/start_go_home",
      std::bind(&WaypointFlyingModule::start_go_home_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  cancel_go_home_srv_ = create_service<Trigger>(
      "psdk_ros2/cancel_go_home",
      std::bind(&WaypointFlyingModule::cancel_go_home_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  obtain_ctrl_authority_srv_ = create_service<Trigger>(
      "psdk_ros2/obtain_ctrl_authority",
      std::bind(&WaypointFlyingModule::obtain_ctrl_authority_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  release_ctrl_authority_srv_ = create_service<Trigger>(
      "psdk_ros2/release_ctrl_authority",
      std::bind(&WaypointFlyingModule::release_ctrl_authority_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  turn_on_motors_srv_ = create_service<Trigger>(
      "psdk_ros2/turn_on_motors",
      std::bind(&WaypointFlyingModule::turn_on_motors_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  turn_off_motors_srv_ = create_service<Trigger>(
      "psdk_ros2/turn_off_motors",
      std::bind(&WaypointFlyingModule::turn_off_motors_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  takeoff_srv_ = create_service<Trigger>(
      "psdk_ros2/takeoff",
      std::bind(&WaypointFlyingModule::start_takeoff_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  land_srv_ = create_service<Trigger>(
      "psdk_ros2/land",
      std::bind(&WaypointFlyingModule::start_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  cancel_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/cancel_landing",
      std::bind(&WaypointFlyingModule::cancel_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_confirm_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/start_confirm_landing",
      std::bind(&WaypointFlyingModule::start_confirm_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  start_force_landing_srv_ = create_service<Trigger>(
      "psdk_ros2/start_force_landing",
      std::bind(&WaypointFlyingModule::start_force_landing_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  set_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_horizontal_vo_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::set_horizontal_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_horizontal_radar_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::set_horizontal_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_upwards_vo_obstacle_avoidance_srv_ = create_service<SetObstacleAvoidance>(
      "psdk_ros2/set_upwards_vo_obstacle_avoidance",
      std::bind(&WaypointFlyingModule::set_upwards_vo_obstacle_avoidance_cb,
                this, std::placeholders::_1, std::placeholders::_2));
  set_upwards_radar_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_upwards_radar_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::set_upwards_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  set_downwards_vo_obstacle_avoidance_srv_ =
      create_service<SetObstacleAvoidance>(
          "psdk_ros2/set_downwards_vo_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::set_downwards_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_horizontal_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_horizontal_vo_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::get_horizontal_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_upwards_vo_obstacle_avoidance_srv_ = create_service<GetObstacleAvoidance>(
      "psdk_ros2/get_upwards_vo_obstacle_avoidance",
      std::bind(&WaypointFlyingModule::get_upwards_vo_obstacle_avoidance_cb,
                this, std::placeholders::_1, std::placeholders::_2));
  get_upwards_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_upwards_radar_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::get_upwards_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_downwards_vo_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_downwards_vo_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::get_downwards_vo_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));
  get_horizontal_radar_obstacle_avoidance_srv_ =
      create_service<GetObstacleAvoidance>(
          "psdk_ros2/get_horizontal_radar_obstacle_avoidance",
          std::bind(
              &WaypointFlyingModule::get_horizontal_radar_obstacle_avoidance_cb,
              this, std::placeholders::_1, std::placeholders::_2));

#endif
  
  return CallbackReturn::SUCCESS;
}

WaypointFlyingModule::CallbackReturn
WaypointFlyingModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating WaypointFlyingModule");
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
    RCLCPP_INFO(get_logger(), "HMS already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating Waypoint Flying");
  wfm_pointer = this;

  T_DjiReturnCode resinit = DjiWaypointV2_Init();

  // std::cerr << "INIT***********************resinit: " << resinit << std::endl;

  if (resinit > 0) {
      is_module_initialized_ = false;
      return false;
  }
  
  is_module_initialized_ = true;
  return true;
}

bool
WaypointFlyingModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing Waypoint Flying");
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


void WaypointFlyingModule::subscribe_waypoint_v2_event_callback(
     const std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2Event::Request> req,
     std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2Event::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "subscribe_waypoint_v2_event_callback");
  
  res->result = true;
}

T_DjiReturnCode WaypointFlyingModule::state_callback(T_DjiWaypointV2MissionStatePush stateData) {

  return 0;
}
    
T_DjiReturnCode state_callback2(T_DjiWaypointV2MissionStatePush stateData) {
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "state_callback2: %d %d %d",
  // stateData.curWaypointIndex, stateData.state, stateData.velocity);

  psdk_interfaces::msg::WaypointV2MissionStatePush msg;
  msg.common_data_version = 0;
  msg.common_data_len = 0;
  msg.cur_waypoint_index = stateData.curWaypointIndex;
  msg.state = stateData.state; // 0x1 mission prepared; 0x2 enter mission
  msg.velocity = stateData.velocity;

  wfm_pointer->state_push_publisher->publish(msg);
  
  return 0;
}
    
T_DjiReturnCode event_callback2(T_DjiWaypointV2MissionEventPush eventData) {
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "state_callback2: %d %d %d",
  // stateData.curWaypointIndex, stateData.state, stateData.velocity);

  psdk_interfaces::msg::WaypointV2MissionEventPush msg;
  msg.common_data_version = 0;
  msg.common_data_len = 0;
  msg.cur_waypoint_index = stateData.curWaypointIndex;
  msg.state = stateData.state; // 0x1 mission prepared; 0x2 enter mission
  msg.velocity = stateData.velocity;

  wfm_pointer->state_push_publisher->publish(msg);
  
  return 0;
}
    
void WaypointFlyingModule::subscribe_waypoint_v2_state_callback(
     const std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2State::Request> req,
     std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2State::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "subscribe_waypoint_v2_state_callback");

  wfm_pointer = this;

  res->result = true;
  /// T_DjiReturnCode regres = DjiWaypointV2_RegisterMissionStateCallback(std::bind(&WaypointFlyingModule::state_callback, this, std::placeholders::_1));
  // T_DjiReturnCode regres = DjiWaypointV2_RegisterMissionStateCallback([&this](T_DjiWaypointV2MissionStatePush stateData){this->state_callback(stateData)});
  T_DjiReturnCode regres = DjiWaypointV2_RegisterMissionStateCallback(&state_callback2);
  if (regres > 0) {
    res->result = false;
  }
}
    
void WaypointFlyingModule::pause_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pause_waypoint_v2_mission_callback");
  
  res->result = true;
}

void WaypointFlyingModule::resume_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::ResumeWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::ResumeWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "resume_waypoint_v2_mission_callback");

  res->result = true;
}

void WaypointFlyingModule::start_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start_waypoint_v2_mission_callback");

  res->result = true;
  T_DjiReturnCode startres = DjiWaypointV2_Start();
  std::cerr << "startres: " << startres << std::endl;

  if (startres > 0) {
    res->result = false;
  }
}

void WaypointFlyingModule::stop_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stop_waypoint_v2_mission_callback");

  res->result = true;
}

void WaypointFlyingModule::download_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::DownloadWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::DownloadWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "download_waypoint_v2_mission_callback");

  res->result = true;
}

void WaypointFlyingModule::upload_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Mission::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "upload_waypoint_v2_mission_callback");

  T_DjiReturnCode uploadres = DjiWaypointV2_UploadMission(ms);
  std::cerr << "uploadres: " << uploadres << std::endl;

  res->result = true;
  
  if (uploadres > 0) {
    res->result = false;
  }


}

void WaypointFlyingModule::generate_waypoint_v2_action_callback(
     const std::shared_ptr<psdk_interfaces::srv::GenerateWaypointV2Action::Request> req,
     std::shared_ptr<psdk_interfaces::srv::GenerateWaypointV2Action::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "generate_waypoint_v2_action_callback");  
  res->result = true;    
}

void WaypointFlyingModule::init_waypoint_v2_setting_callback(
     const std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Request> request,
     std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Response> response) {

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "init_waypoint_v2_setting_callback");


  uint16_t polygonNum = request->polygon_num;
  float radius = request->radius;

  uint16_t actionNum = request->action_num;
  srand(int(time(0)));
  
  ms = new T_DjiWayPointV2MissionSettings();

  ms->missionID = rand(); // uint32_t
  ms->repeatTimes = request->waypoint_v2_init_settings.repeat_times; // No repeat, 1 = execute to times
  /// ms->finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
  ms->finishedAction = static_cast<E_DJIWaypointV2MissionFinishedAction>(request->waypoint_v2_init_settings.finished_action);
  ms->maxFlightSpeed = request->waypoint_v2_init_settings.max_flight_speed;
  ms->autoFlightSpeed = request->waypoint_v2_init_settings.auto_flight_speed;
  ms->actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
  // ms->gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
  // DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT
  ms->gotoFirstWaypointMode = static_cast<E_DJIWaypointV2MissionGotoFirstWaypointMode>(request->waypoint_v2_init_settings.goto_first_waypoint_mode);
  ms->missTotalLen = request->waypoint_v2_init_settings.mission.size();

  ms->mission = (T_DjiWaypointV2 *) malloc(ms->missTotalLen*sizeof(T_DjiWaypointV2));

  for (uint16_t i = 0; i < request->waypoint_v2_init_settings.mission.size(); i++) {
    T_DjiWaypointV2 wp;

    wp.longitude = request->waypoint_v2_init_settings.mission[i].longitude;
    wp.latitude = request->waypoint_v2_init_settings.mission[i].latitude;
    wp.relativeHeight = request->waypoint_v2_init_settings.mission[i].relative_height;
    wp.waypointType = static_cast<E_DJIWaypointV2FlightPathMode>(request->waypoint_v2_init_settings.mission[i].waypoint_type);
    /// wp.waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
    wp.headingMode = static_cast<E_DJIWaypointV2HeadingMode>(request->waypoint_v2_init_settings.mission[i].heading_mode);    
    /// wp.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
    wp.config.useLocalCruiseVel = request->waypoint_v2_init_settings.mission[i].config.use_local_cruise_vel;
    wp.config.useLocalMaxVel = request->waypoint_v2_init_settings.mission[i].config.use_local_max_vel;
    wp.dampingDistance = request->waypoint_v2_init_settings.mission[i].damping_distance;
    wp.heading = request->waypoint_v2_init_settings.mission[i].heading;
    wp.turnMode = static_cast<E_DJIWaypointV2TurnMode>(request->waypoint_v2_init_settings.mission[i].turn_mode);  
    /// wp.turnMode = DJI_WAYPOINT_V2_TURN_MODE_UNKNOWN;
    wp.maxFlightSpeed = request->waypoint_v2_init_settings.mission[i].max_flight_speed;
    wp.autoFlightSpeed = request->waypoint_v2_init_settings.mission[i].auto_flight_speed;
    ms->mission[i] = wp;
  }
  
  ///ms->mission =  // T_DjiWaypointV2 *mission;

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
  //  T_DjiReturnCode speedres = DjiWaypointV2_SetGlobalCruiseSpeed(cruise_speed);
  //  std::cerr << "speedres: " << speedres << std::endl;
  
  response->result = true;
  //if (speedres > 0) {
  //    response->result = false;
  //  }
}

void WaypointFlyingModule::upload_waypoint_v2_action_callback(
     const std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Request> req,
     std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "upload_waypoint_v2_action_callback");  
  res->result = true;      
}



}
