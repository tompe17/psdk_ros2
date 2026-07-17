/*
 * Copyright (C) 2024
 *
 * @file coord.cpp
 */

#include "psdk_wrapper/modules/coord.hpp"

#include <dji_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

#include "psdk_wrapper/modules/camera.hpp"
#include "psdk_wrapper/modules/liveview.hpp"

using namespace std::placeholders;

namespace psdk_ros2
{

CoordModule::CoordModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().use_intra_process_comms(true).arguments(
              {"--ros-args", "-r", name + ":__node:=" + name}))
{
  RCLCPP_INFO(get_logger(), "Creating CoordModule");

  declare_parameter("location", "granso");
  get_parameter("location", location_);

  RCLCPP_INFO(get_logger(), "Location: %s", location_.c_str());


}

CoordModule::~CoordModule()
{
  RCLCPP_INFO(get_logger(), "Destroying CoordModule");
}

/*****************************************************************************/
/* Lifecycle                                                                 */
/*****************************************************************************/

CoordModule::CallbackReturn
CoordModule::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring CoordModule");

  // widget_command_pub_ =
  // create_publisher<std_msgs::msg::String>("psdk_ros2/widget_command", 10);

  // widget_text_sub_ = create_subscription<std_msgs::msg::String>(
  // "psdk_ros2/rc_display_text", 10,
  // std::bind(&CoordModule::widget_text_callback, this,
  // std::placeholders::_1));

  ct_ = new CoordTrans(location_);
  double lon, lat;
  ct_->world_to_wgs84(0.0, 0.0, 0.0, lon, lat, world_origin_elevation_);
  RCLCPP_INFO(get_logger(), "World origin elevation: %g", world_origin_elevation_);

  return CallbackReturn::SUCCESS;
}

CoordModule::CallbackReturn
CoordModule::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating CoordModule");

  return CallbackReturn::SUCCESS;
}

CoordModule::CallbackReturn
CoordModule::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating CoordModule");

  return CallbackReturn::SUCCESS;
}

CoordModule::CallbackReturn
CoordModule::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning CoordModule");

  // widget_command_pub_.reset();

  return CallbackReturn::SUCCESS;
}

CoordModule::CallbackReturn
CoordModule::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down CoordModule");

  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
/* Init / Deinit                                                             */
/*****************************************************************************/

bool
CoordModule::init()
{
  RCLCPP_INFO(get_logger(), "Coord module initialized");

  return true;
}

bool
CoordModule::deinit()
{
  return true;
}

// CoordModule::set_takeoff_info(const lrs_m300_msgs::msg::TakeoffInfo & msg)
// {
// takeoff_height_above_ellipsoid = msg.height_above_ellipsoid;
// takeoff_gps_altitude = msg.gps_altitude;
// have_takeoff_info = true;
// }

}  // namespace psdk_ros2