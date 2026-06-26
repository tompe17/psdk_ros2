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
#include "psdk_wrapper/modules/liveview.hpp"

using namespace std::placeholders;

namespace psdk_ros2
{

/*****************************************************************************/
/* Static widget callback declarations                                       */
/*****************************************************************************/

T_DjiWidgetHandlerListItem WidgetModule::widget_handlers_[] = {
    {0, DJI_WIDGET_TYPE_LIST, WidgetModule::camera_lens_set_value,
     WidgetModule::camera_lens_get_value, nullptr},
    {1, DJI_WIDGET_TYPE_SWITCH, WidgetModule::streaming_state_set,
     WidgetModule::streaming_state_get, nullptr}};

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
  widget_handlers_[1].userData = this;

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



  //  std::cout << "camera_lens_set_value: index=" << widgetIndex
  //            << " value=" << widgetValue << " current_lens_" <<
  //            (int)self->current_lens_ << std::endl;

  switch (widgetValue)
  {
    case 0:
      std::cout << "wide" << std::endl;
      self->handleWidePressed();
      break;

    case 1:
      std::cout << "zoom" << std::endl;
      self->handleZoomPressed();
      break;
    case 2:
      self->handleThermalPressed();
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

  // streaming source (camera lens)
  if (widgetIndex == 0)
  {
    auto camera_source =  psdk_ros2::global_liveview_ptr_->get_camera_source_index();
    if (camera_source==-1)
      return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    *widgetValue = camera_source;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool
WidgetModule::display_text(const std::string &text)
{
  auto djiStat = DjiWidgetFloatingWindow_ShowMessage(text.c_str());
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Floating window show message error, stat = 0x%08llX",
                   djiStat);
  }
}

T_DjiReturnCode
WidgetModule::streaming_state_set(E_DjiWidgetType type, uint32_t index,
                                  int32_t value, void *user_data)
{
  if (type == DJI_WIDGET_TYPE_SWITCH && index == 1)
  {
    auto *self = static_cast<WidgetModule *>(user_data);

    if (self == nullptr) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    bool streaming = psdk_ros2::global_liveview_ptr_->is_streaming();

    RCLCPP_INFO(self->get_logger(), "Currently streaming: %d", streaming);
    if (streaming)
    {
      RCLCPP_INFO(self->get_logger(), "Requesting stop streaming");
      std::thread([liveview = psdk_ros2::global_liveview_ptr_]
                  { liveview->camera_setup_streaming(false, -1, -1, true); })
          .detach();

      //      self->display_text("Stopping.");
      //      StartStreaming();
    }
    else
    {
      RCLCPP_INFO(self->get_logger(), "Stop streaming pressed");

      //      StopStreaming();}
    }
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
WidgetModule::streaming_state_get(E_DjiWidgetType type, uint32_t index,
                                  int32_t *value, void *user_data)
{
  //  std::cout << "streaming_state_get" << std::endl;

  if (type == DJI_WIDGET_TYPE_SWITCH && index == 1)
  {
    auto *self = static_cast<WidgetModule *>(user_data);

    if (self == nullptr) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    bool streaming = psdk_ros2::global_liveview_ptr_->is_streaming();

    *value = streaming ? 1 : 0;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
WidgetModule::handleWidePressed()
{
  RCLCPP_INFO(get_logger(), "handleWidePressed");

  std::thread(
      [liveview = psdk_ros2::global_liveview_ptr_]
      {
        liveview->camera_setup_streaming(false, -1, -1, true);
        liveview->camera_setup_streaming(true, 1, 1, true);
      })
      .detach();
}

void
WidgetModule::handleZoomPressed()
{
  RCLCPP_INFO(get_logger(), "handleZoomPressed");

  // stop previous
  std::thread(
      [liveview = psdk_ros2::global_liveview_ptr_]
      {
        liveview->camera_setup_streaming(false, -1, -1, true);
        liveview->camera_setup_streaming(true, 1, 2, true);
      })
      .detach();

  RCLCPP_ERROR(get_logger(), "Finished");

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
WidgetModule::handleThermalPressed()
{
  RCLCPP_INFO(get_logger(), "handleThermalPressed");

  //  DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE = 1,
  //  DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM = 2,
  //  DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR = 3,

  // stop previous
  std::thread(
      [liveview = psdk_ros2::global_liveview_ptr_]
      {
        liveview->camera_setup_streaming(false, -1, -1, true);
        liveview->camera_setup_streaming(true, 1, 3, true);
      })
      .detach();

  //  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //  std::thread([liveview = psdk_ros2::global_liveview_ptr_]
  //              { liveview->camera_setup_streaming(true, 1, 3, true); })
  //      .detach();

  //  std::thread([camera = psdk_ros2::global_camera_ptr_] {
  //  camera->camera_set_optical_zoom(1, 5.0); })
  //      .detach();
  RCLCPP_ERROR(get_logger(), "Finished");
}

void
WidgetModule::publishCommand(const std::string &command)
{
}

}  // namespace psdk_ros2