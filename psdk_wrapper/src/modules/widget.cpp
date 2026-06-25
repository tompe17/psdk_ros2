/*
 * Copyright (C) 2024
 *
 * @file widget.cpp
 */

#include "psdk_wrapper/modules/widget.hpp"

#include <dji_logger.h>

using namespace std::placeholders;

namespace psdk_ros2
{

// std::shared_ptr<WidgetModule> global_widget_ptr_;

/*****************************************************************************/
/* Static widget callback declarations                                       */
/*****************************************************************************/

T_DjiWidgetHandlerListItem WidgetModule::widget_handlers_[] = {
    {CAMERA_LENS_WIDGET_INDEX, DJI_WIDGET_TYPE_LIST,
     WidgetModule::camera_lens_set_value, WidgetModule::camera_lens_get_value,
     nullptr}};

/*****************************************************************************/
/* Constructor / Destructor                                                  */
/*****************************************************************************/

WidgetModule::WidgetModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().use_intra_process_comms(true).arguments(
              {"--ros-args", "-r", name + ":__node:=" + name}))
{
  RCLCPP_INFO(get_logger(), "Creating WidgetModule");
}

WidgetModule::~WidgetModule()
{
  RCLCPP_INFO(get_logger(), "Destroying WidgetModule");
}

/*****************************************************************************/
/* Lifecycle                                                                 */
/*****************************************************************************/

WidgetModule::CallbackReturn
WidgetModule::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring WidgetModule");

  widget_command_pub_ =
      create_publisher<std_msgs::msg::String>("psdk_ros2/widget_command", 10);

  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating WidgetModule");

  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating WidgetModule");

  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning WidgetModule");

  widget_command_pub_.reset();

  return CallbackReturn::SUCCESS;
}

WidgetModule::CallbackReturn
WidgetModule::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down WidgetModule");

  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
/* Init / Deinit                                                             */
/*****************************************************************************/

bool
WidgetModule::init()
{
  T_DjiReturnCode rc;

  rc = DjiWidget_Init();

  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "DjiWidget_Init failed (%ld)", rc);

    return false;
  }

  rc = DjiWidget_RegDefaultUiConfigByDirPath("cfg/widget_file/en");

  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to register widget UI (%ld)", rc);

    return false;
  }

  rc = DjiWidget_RegHandlerList(
      widget_handlers_, sizeof(widget_handlers_) / sizeof(widget_handlers_[0]));

  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to register widget handlers (%ld)", rc);

    return false;
  }

  RCLCPP_INFO(get_logger(), "Widget module initialized");

  return true;
}

bool
WidgetModule::deinit()
{
  return true;
}

/*****************************************************************************/
/* Widget callbacks                                                          */
/*****************************************************************************/

T_DjiReturnCode
WidgetModule::camera_lens_set_value(E_DjiWidgetType widgetType,
                                    uint32_t widgetIndex, int32_t widgetValue,
                                    void *userData)
{
  (void)widgetType;
  (void)widgetIndex;

  auto *self = static_cast<WidgetModule *>(userData);

  if (self == nullptr) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  self->current_lens_ = static_cast<LensSelection>(widgetValue);

  switch (self->current_lens_)
  {
    case LensSelection::WIDE:
      self->handleWidePressed();
      break;

    case LensSelection::ZOOM:
      self->handleZoomPressed();
      break;

    default:
      break;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
WidgetModule::camera_lens_get_value(E_DjiWidgetType widgetType,
                                    uint32_t widgetIndex, int32_t *widgetValue,
                                    void *userData)
{
  (void)widgetType;
  (void)widgetIndex;

  auto *self = static_cast<WidgetModule *>(userData);

  if (self == nullptr || widgetValue == nullptr)
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  *widgetValue = static_cast<int32_t>(self->current_lens_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
WidgetModule::handleWidePressed()
{
  RCLCPP_INFO(get_logger(), "Wide selected");
  publishCommand("wide");
}

void
WidgetModule::handleZoomPressed()
{
  RCLCPP_INFO(get_logger(), "Zoom selected");
  publishCommand("zoom");
}

void WidgetModule::publishCommand(const std::string &command)
{

}


}  // namespace psdk_ros2