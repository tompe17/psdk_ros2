/*
* Copyright (C) 2024
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0.
 */

/**
* @file widget.hpp
*
* @brief Header file for the WidgetModule class
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_

#include <dji_widget.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <std_msgs/msg/string.hpp>

namespace psdk_ros2
{

class WidgetModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WidgetModule(const std::string &name);

  ~WidgetModule();

  CallbackReturn on_configure(
      const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_activate(
      const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &state) override;

  bool init();
  bool deinit();

 private:
  static constexpr uint32_t CAMERA_LENS_WIDGET_INDEX = 0;

  enum class LensSelection
  {
    WIDE = 0,
    ZOOM = 1
  };

  /*
  * DJI widget callbacks
   */
  static T_DjiReturnCode camera_lens_set_value(
      E_DjiWidgetType widgetType,
      uint32_t widgetIndex,
      int32_t widgetValue,
      void *userData);

  static T_DjiReturnCode camera_lens_get_value(
      E_DjiWidgetType widgetType,
      uint32_t widgetIndex,
      int32_t *widgetValue,
      void *userData);

  /*
  * Internal handlers
   */
  void handleWidePressed();
  void handleZoomPressed();

  void publishCommand(const std::string &command);

  /*
  * Widget state
   */
  LensSelection current_lens_{LensSelection::WIDE};

  /*
  * ROS
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      widget_command_pub_;

  /*
  * DJI widget handler table
   */
  static T_DjiWidgetHandlerListItem widget_handlers_[];
};

extern std::shared_ptr<WidgetModule> global_widget_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_