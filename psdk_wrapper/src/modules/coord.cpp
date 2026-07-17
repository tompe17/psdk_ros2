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


}  // namespace psdk_ros2