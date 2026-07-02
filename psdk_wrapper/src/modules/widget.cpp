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
    {0, DJI_WIDGET_TYPE_LIST, WidgetModule::widget_state_set,
     WidgetModule::widget_state_get, nullptr},
    {1, DJI_WIDGET_TYPE_SWITCH, WidgetModule::widget_state_set,
     WidgetModule::widget_state_get, nullptr},
    {2, DJI_WIDGET_TYPE_SCALE, WidgetModule::widget_state_set,
     WidgetModule::widget_state_get, nullptr},
    {3, DJI_WIDGET_TYPE_SCALE, WidgetModule::widget_state_set,
     WidgetModule::widget_state_get, nullptr}
};

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

  widget_text_sub_ = create_subscription<std_msgs::msg::String>(
      "psdk_ros2/rc_display_text", 10,
      std::bind(&WidgetModule::widget_text_callback, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

void
WidgetModule::widget_text_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Widget text: %s", msg->data.c_str());

  display_text(msg->data);
}

WidgetModule::CallbackReturn
WidgetModule::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating WidgetModule");

  //  widget_timer_ = create_wall_timer(
  //      std::chrono::seconds(1),
  //      std::bind(&WidgetModule::poll_widget_channel, this));

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

  for (auto &handler : widget_handlers_) {
    handler.userData = this;
    // Use handler
  }

  // widget_handlers_[0].userData = this;
  // widget_handlers_[1].userData = this;

  std::string widget_path =
      ament_index_cpp::get_package_share_directory("psdk_wrapper") +
      "/cfg/widget_file/en";

  auto camera_type = psdk_ros2::global_camera_ptr_->get_attached_camera_type();

  switch (camera_type){
    case DJI_CAMERA_TYPE_H20T:
      widget_path+="/h20t";
      break;
    case DJI_CAMERA_TYPE_P1:
      widget_path+="/p1";
      break;
    default:
      RCLCPP_INFO(get_logger(), "Widget module initialized");
      return true;

  }

  //  rc = DjiWidget_RegDefaultUiConfigByDirPath(
  //      "/home/lrs/lrs_jazzy/src/psdk_ros2/psdk_wrapper/cfg/widget_file/en/");
  rc = DjiWidget_RegDefaultUiConfigByDirPath(widget_path.c_str());
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to register widget UI (%ld)", rc);

    return false;
  }
  RCLCPP_INFO(get_logger(), "Widget path: %s", widget_path.c_str());
//  RCLCPP_INFO(get_logger(), "RegDefaultUiConfig returned %ld", rc);

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

  auto *self = static_cast<WidgetModule *>(userData);

  if (self == nullptr || widgetValue == nullptr)
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  // streaming source (camera lens)
  if (widgetIndex == 0)
  {
    auto camera_source =
        psdk_ros2::global_liveview_ptr_->get_camera_source_index();
    if (camera_source == -1) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

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
    return false;
  }
  return true;
}

T_DjiReturnCode
WidgetModule::widget_state_set(E_DjiWidgetType type, uint32_t index,
                               int32_t value, void *user_data)
{
  auto *self = static_cast<WidgetModule *>(user_data);
  if (self == nullptr) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  switch (index)
  {
    case 0:
    {
      switch (value)
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
      break;
    }
    case 1:
      //      if (type == DJI_WIDGET_TYPE_SWITCH && index == 1)
      {
        bool streaming = psdk_ros2::global_liveview_ptr_->is_streaming();

        RCLCPP_INFO(self->get_logger(), "Currently streaming: %d", streaming);

//        std::thread(
//            [camera = psdk_ros2::global_camera_ptr_]
//            {
//              T_DjiCameraManagerLaserRangingInfo laser_info;
//              laser_info.enable_lidar = true;
//              camera->camera_get_laser_ranging_info(1, laser_info);
//            })
//            .detach();

        //        std::thread(
        //            [camera = psdk_ros2::global_camera_ptr_]
        //            {
        //              T_DjiCameraManagerVideoFormat video_format;
        //
        //              camera->camera_get_video_resolution_frame_rate(1,
        //              video_format); std::cout << "Resolution: " <<
        //              video_format.videoResolution
        //                        << "rate: " << video_format.videoFrameRate <<
        //                        std::endl;
        //            })
        //            .detach();

        //        std::thread([camera = psdk_ros2::global_camera_ptr_]
        //                    {
        //                    camera->camera_set_synchronized_split_screen_zoom(2,
        //                    true); })
        //            .detach();
        //        std::thread([camera = psdk_ros2::global_camera_ptr_]
        //                    {
        //                    camera->camera_set_synchronized_split_screen_zoom(3,
        //                    true); })
        //            .detach();

        std::thread(
            [liveview = psdk_ros2::global_liveview_ptr_, streaming]
            { liveview->camera_setup_streaming(!streaming, -1, -1, true); })
            .detach();

        if (streaming)
          self->display_text("Now NOT streaming...");
        else
          self->display_text("Now streaming...");

        break;
      }
    case 2:
      RCLCPP_INFO(self->get_logger(), "Setting jpeg %d", value);

      break;

    default:

      RCLCPP_INFO(self->get_logger(), "Unknown widget index: %d", index);
      break;
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
WidgetModule::widget_state_get(E_DjiWidgetType type, uint32_t index,
                               int32_t *value, void *user_data)
{
  auto *self = static_cast<WidgetModule *>(user_data);
  if (self == nullptr) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  switch (index)
  {
    case 0:
    {
      auto camera_source =
          psdk_ros2::global_liveview_ptr_->get_camera_source_index();
      if (camera_source == -1) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

      *value = camera_source;
      break;
    }
    case 1:
    {  //      if (type == DJI_WIDGET_TYPE_SWITCH)

      bool streaming = psdk_ros2::global_liveview_ptr_->is_streaming();

      *value = streaming ? 1 : 0;
      break;
    }
    case 2:
    case 3:
    {
      *value = psdk_ros2::global_liveview_ptr_->get_image_jpeg_compression();

      break;
    }
    default:
      RCLCPP_INFO(self->get_logger(),
                  "widget_state_get: unknown widget index: %d", index);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
WidgetModule::poll_widget_channel()
{
  T_DjiDataChannelState state;

  if (DjiWidgetFloatingWindow_GetChannelState(&state) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    return;

//  std::cout << "poll_widget_channel: " << static_cast<int>(state) <<
//  std::endl;
#if 0
  if (state == /* connected value */ && !widget_registered)
  {
    DjiWidget_RegDefaultUiConfigByDirPath(widget_path.c_str());
    DjiWidget_RegHandlerList(widget_handlers_,
                             sizeof(widget_handlers_) /
                                 sizeof(widget_handlers_[0]));

    widget_registered = true;
  }

  if (state != /* connected value */)
  {
    widget_registered = false;
  }
#endif
}

void
WidgetModule::handleWidePressed()
{
  RCLCPP_INFO(get_logger(), "handleWidePressed");

  std::thread(
      [liveview = psdk_ros2::global_liveview_ptr_]
      {
        liveview->camera_setup_streaming(false, -1, -1, true);
        liveview->camera_setup_streaming(
            true, 1, DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE, true);
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
        liveview->camera_setup_streaming(
            true, 1, DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM, true);
      })
      .detach();

  RCLCPP_ERROR(get_logger(), "Finished");
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
        liveview->camera_setup_streaming(
            true, 1, DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR, true);
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