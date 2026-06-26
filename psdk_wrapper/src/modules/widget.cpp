/*
 * Copyright (C) 2024
 *
 * @file widget.cpp
 */

#include "psdk_wrapper/modules/widget.hpp"

#include <dji_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

#include "psdk_wrapper/modules/camera.hpp"

using namespace std::placeholders;

namespace psdk_ros2
{

/*****************************************************************************/
/* Static widget callback declarations                                       */
/*****************************************************************************/

T_DjiWidgetHandlerListItem WidgetModule::widget_handlers_[] = {
    {0, DJI_WIDGET_TYPE_LIST, WidgetModule::camera_lens_set_value,
     WidgetModule::camera_lens_get_value, nullptr}};
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
  camera_ = nullptr;
}

WidgetModule::~WidgetModule()
{
  RCLCPP_INFO(get_logger(), "Destroying WidgetModule");
}

void
WidgetModule::setCameraModule(std::shared_ptr<CameraModule> camera)
{
  camera_ = camera;
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

  std::cout << "INIT" << std::endl;

  rc = DjiWidget_Init();
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "DjiWidget_Init failed (%ld)", rc);

    return false;
  }

  widget_handlers_[0].userData = this;

  std::string widget_path =
      ament_index_cpp::get_package_share_directory("psdk_wrapper") +
      "/cfg/widget_file/en";

  rc = DjiWidget_RegDefaultUiConfigByDirPath(widget_path.c_str());

  //  rc = DjiWidget_RegDefaultUiConfigByDirPath(
  //      "/home/lrs/lrs_jazzy/src/psdk_ros2/psdk_wrapper/cfg/widget_file/en/");

  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to register widget UI (%ld)", rc);

    return false;
  }
  RCLCPP_INFO(get_logger(), "Widget path: %s", widget_path.c_str());
  RCLCPP_INFO(get_logger(), "RegDefaultUiConfig returned %ld", rc);

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

  //  std::cout << "camera_lens_set_value: index=" << widgetIndex
  //            << " value=" << widgetValue << " current_lens_" <<
  //            (int)self->current_lens_ << std::endl;

  switch (self->current_lens_)
  {
    case LensSelection::WIDE:
      std::cout << "wide" << std::endl;
      self->handleWidePressed();
      break;

    case LensSelection::ZOOM:
      std::cout << "zoom" << std::endl;
      self->handleZoomPressed();
      break;
    case LensSelection::THERMAL:
      //      self->handleZoomPressed();
      std::cout << "thermal" << std::endl;
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
  RCLCPP_INFO(get_logger(), "handleWidePressed");
  if (camera_ != nullptr)
  {
    auto res = camera_->camera_set_optical_zoom(1, 2.0);
    if (!res)
    {
      RCLCPP_ERROR(get_logger(), "Wide not selected");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Wide selected");
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "camera_ not set");
  }
}

void
WidgetModule::handleZoomPressed()
{
  RCLCPP_INFO(get_logger(), "handleZoomPressed");

  std::thread([camera = camera_] { camera->camera_set_optical_zoom(1, 5.0); })
      .detach();
  RCLCPP_ERROR(get_logger(), "Zoom selected");

#if 0
//  if (camera_ != nullptr)
  {
    auto res = psdk_ros2::global_camera_ptr_->camera_set_optical_zoom(1, 5.0);

//    auto res = camera_->camera_set_optical_zoom(1, 5.0);
    if (!res)
    {
      RCLCPP_ERROR(get_logger(), "Zoom not selected");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Zoom selected");
    }
  }
//  else {
//    RCLCPP_ERROR(get_logger(), "camera_ not set");
//  }
  RCLCPP_ERROR(get_logger(), "finished");
#endif
}

void
WidgetModule::publishCommand(const std::string &command)
{
}

}  // namespace psdk_ros2