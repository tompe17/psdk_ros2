/**
 * @file waypoint_flying.hpp
 *
 * @brief Header file for the WaypointFlyingModule class
 *
 * @authors Tommy Persson
 * Contact: tommy.persson@liu.se
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_FLYING_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_FLYING_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "std_srvs/srv/trigger.hpp"

#include "psdk_interfaces/srv/download_waypoint_v2_mission.hpp"
#include "psdk_interfaces/srv/generate_waypoint_v2_action.hpp"
#include "psdk_interfaces/srv/init_waypoint_v2_setting.hpp"
#include "psdk_interfaces/srv/pause_waypoint_v2_mission.hpp"
#include "psdk_interfaces/srv/resume_waypoint_v2_mission.hpp"
#include "psdk_interfaces/srv/start_waypoint_v2_mission.hpp"
#include "psdk_interfaces/srv/stop_waypoint_v2_mission.hpp"
#include "psdk_interfaces/srv/subscribe_waypoint_v2_event.hpp"
#include "psdk_interfaces/srv/subscribe_waypoint_v2_state.hpp"
#include "psdk_interfaces/srv/upload_waypoint_v2_action.hpp"
#include "psdk_interfaces/srv/upload_waypoint_v2_mission.hpp"

#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

  class WaypointFlyingModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using Trigger = std_srvs::srv::Trigger;
  //  using SetHomeFromGPS = psdk_interfaces::srv::SetHomeFromGPS;
  //  using SetGoHomeAltitude = psdk_interfaces::srv::SetGoHomeAltitude;
  //  using GetGoHomeAltitude = psdk_interfaces::srv::GetGoHomeAltitude;
  //  using SetObstacleAvoidance = psdk_interfaces::srv::SetObstacleAvoidance;
  //  using GetObstacleAvoidance = psdk_interfaces::srv::GetObstacleAvoidance;

  rclcpp::Service<psdk_interfaces::srv::SubscribeWaypointV2Event>::SharedPtr subscribe_waypoint_v2_event_service;
  rclcpp::Service<psdk_interfaces::srv::SubscribeWaypointV2State>::SharedPtr subscribe_waypoint_v2_state_service;
  rclcpp::Service<psdk_interfaces::srv::PauseWaypointV2Mission>::SharedPtr pause_waypoint_v2_mission_service;    
  rclcpp::Service<psdk_interfaces::srv::ResumeWaypointV2Mission>::SharedPtr resume_waypoint_v2_mission_service;    
  rclcpp::Service<psdk_interfaces::srv::StopWaypointV2Mission>::SharedPtr stop_waypoint_v2_mission_service;    
  rclcpp::Service<psdk_interfaces::srv::StartWaypointV2Mission>::SharedPtr start_waypoint_v2_mission_service;    
  rclcpp::Service<psdk_interfaces::srv::DownloadWaypointV2Mission>::SharedPtr download_waypoint_v2_mission_service;    
  rclcpp::Service<psdk_interfaces::srv::UploadWaypointV2Mission>::SharedPtr upload_waypoint_v2_mission_service;    

  rclcpp::Service<psdk_interfaces::srv::GenerateWaypointV2Action>::SharedPtr generate_waypoint_v2_action_service;    
  rclcpp::Service<psdk_interfaces::srv::InitWaypointV2Setting>::SharedPtr init_waypoint_v2_setting_service;    
  rclcpp::Service<psdk_interfaces::srv::UploadWaypointV2Action>::SharedPtr upload_waypoint_v2_action_service;
  
  void subscribe_waypoint_v2_event_callback(
     const std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2Event::Request> req,
     std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2Event::Response> res);

  void subscribe_waypoint_v2_state_callback(
     const std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2State::Request> req,
     std::shared_ptr<psdk_interfaces::srv::SubscribeWaypointV2State::Response> res);
    
  void pause_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::PauseWaypointV2Mission::Response> res);

  void resume_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::ResumeWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::ResumeWaypointV2Mission::Response> res);

  void start_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::StartWaypointV2Mission::Response> res);

  void stop_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::StopWaypointV2Mission::Response> res);

  void download_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::DownloadWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::DownloadWaypointV2Mission::Response> res);

  void upload_waypoint_v2_mission_callback(
     const std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Mission::Request> req,
     std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Mission::Response> res);

  void generate_waypoint_v2_action_callback(
     const std::shared_ptr<psdk_interfaces::srv::GenerateWaypointV2Action::Request> req,
     std::shared_ptr<psdk_interfaces::srv::GenerateWaypointV2Action::Response> res);

  void init_waypoint_v2_setting_callback(
     const std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Request> req,
     std::shared_ptr<psdk_interfaces::srv::InitWaypointV2Setting::Response> res);

  void upload_waypoint_v2_action_callback(
     const std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Request> req,
     std::shared_ptr<psdk_interfaces::srv::UploadWaypointV2Action::Response> res);
  
  /**
   * @brief Construct a new WaypointFlyingModule object
   * @param node_name Name of the node
   */
  explicit WaypointFlyingModule(const std::string &name);

  /**
   * @brief Destroy the Flight Control Module object
   */
  ~WaypointFlyingModule();

  /**
   * @brief Configures the flight control module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);

  /**
   * @brief Activates the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  /**
   * @brief Cleans the flight control module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
  /**
   * @brief Deactivates the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  /**
   * @brief Shuts down the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  };

}



#endif
