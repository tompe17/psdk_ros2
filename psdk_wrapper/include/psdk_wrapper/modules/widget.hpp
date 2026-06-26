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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "psdk_wrapper/modules/camera.hpp"

namespace psdk_ros2
{
class CameraModule;
class WidgetModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WidgetModule(const std::string &name);

  ~WidgetModule();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  bool init();
  bool deinit();
  bool display_text(const std::string &text);

  void setCameraModule(std::shared_ptr<CameraModule> camera);

 private:
  static constexpr uint32_t CAMERA_LENS_WIDGET_INDEX = 0;

 private:
  std::shared_ptr<CameraModule> camera_;

  /*
   * DJI widget callbacks
   */
  static T_DjiReturnCode camera_lens_set_value(E_DjiWidgetType widgetType,
                                               uint32_t widgetIndex,
                                               int32_t widgetValue,
                                               void *userData);

  static T_DjiReturnCode camera_lens_get_value(E_DjiWidgetType widgetType,
                                               uint32_t widgetIndex,
                                               int32_t *widgetValue,
                                               void *userData);

  static T_DjiReturnCode streaming_state_set(E_DjiWidgetType type,
                                             uint32_t index, int32_t value,
                                             void *user_data);

  static T_DjiReturnCode streaming_state_get(E_DjiWidgetType type,
                                             uint32_t index, int32_t *value,
                                             void *user_data);

  /*
   * Internal handlers
   */
  void handleWidePressed();
  void handleZoomPressed();
  void handleThermalPressed();

  void publishCommand(const std::string &command);

  /*
   * Widget state
   */

  /*
   * ROS
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr widget_command_pub_;

  /*
   * DJI widget handler table
   */
  static T_DjiWidgetHandlerListItem widget_handlers_[];
};

extern std::shared_ptr<WidgetModule> global_widget_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WIDGET_HPP_