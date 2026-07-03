/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file liveview.cpp
 *
 * @brief Liveview module implementation. This module is responsible for
 * handling the liveview stream from the drone's cameras.
 *
 * @authors Lidia de la Torre Vazquez, Bianca Bendris
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/modules/liveview.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
// #include "image_transport/image_transport.h"
#include <cmath>

#include "psdk_wrapper/modules/camera.hpp"
extern "C"
{
#include <libavutil/log.h>
}

namespace psdk_ros2
{

LiveviewModule::LiveviewModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating LiveviewModule");
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, "dummy");
}

LiveviewModule::~LiveviewModule()
{
  RCLCPP_INFO(get_logger(), "Destroying LiveviewModule");
}

sensor_msgs::msg::CameraInfo
LiveviewModule::get_camera_info(E_DjiCameraType camera_type,
                                E_DjiLiveViewCameraSource source,
                                uint32_t image_width, uint32_t image_height,
                                float zoom_factor)
{
  sensor_msgs::msg::CameraInfo camera_info;

  if (camera_type == DJI_CAMERA_TYPE_H20T)
  {
    // all required files are loaded
    if (camera_infos_.size() == kCameraCalibrationFiles.size())
    {
      if (source == DJI_LIVEVIEW_CAMERA_SOURCE_H20N_WIDE ||
          source == DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT)
      {
        camera_info = camera_infos_[CAMERA_INFO_H20T_WIDE];
      }
      else if (source == DJI_LIVEVIEW_CAMERA_SOURCE_H20N_ZOOM)
      {
        camera_info = get_camera_info_zoom(zoom_factor);
      }
    }
  }
  else if (camera_type == DJI_CAMERA_TYPE_P1)
  {
    camera_info = camera_infos_[CAMERA_INFO_P1];
  }

  // modify the params is image size is different from calibration
  if (image_width != camera_info.width || image_height != camera_info.height)
  {
    double sx = static_cast<double>(image_width) / camera_info.width;
    double sy = static_cast<double>(image_height) / camera_info.height;
    // K
    camera_info.k[0] *= sx;  // fx
    camera_info.k[2] *= sx;  // cx
    camera_info.k[4] *= sy;  // fy
    camera_info.k[5] *= sy;  // cy

    // P
    camera_info.p[0] *= sx;  // fx
    camera_info.p[2] *= sx;  // cx
    camera_info.p[5] *= sy;  // fy
    camera_info.p[6] *= sy;  // cy

    camera_info.width = image_width;
    camera_info.height = image_height;
  }

  return camera_info;
}

bool
LiveviewModule::load_camera_info_files(const std::string &folder)
{
  auto file_path = "file://" + folder + "/";

  for (const auto &[type, filename] : kCameraCalibrationFiles)
  {
    const std::string calibration_file = file_path + filename;

    //    RCLCPP_INFO(get_logger(), "Loading camera calibration file: %s",
    //                calibration_file.c_str());

    auto it = kCameraCalibrationNames.find(type);

    if (it != kCameraCalibrationNames.end())
    {
      const std::string &name = it->second;
      camera_info_manager_->setCameraName(name);
      //      RCLCPP_INFO(get_logger(), "Camera name: %s", name.c_str());
    }

    if (!camera_info_manager_->loadCameraInfo(calibration_file))
    {
      RCLCPP_WARN(get_logger(), "Failed to load camera calibration: %s",
                  calibration_file.c_str());
      return false;
    }

    camera_infos_[type] = camera_info_manager_->getCameraInfo();
  }
  return true;
}

/**
 * Generates camera intrinsics for the current H20T optical zoom.
 *
 * The H20T was calibrated at 2×, 5× and 10× optical zoom. Analysis of
 * repeated calibrations showed:
 *
 *  - The focal length (fx, fy) is highly repeatable and scales almost
 *    linearly with the reported optical zoom.
 *  - The principal point (cx, cy) is reasonably stable up to 5× but
 *    becomes inconsistent at higher zooms, likely due to DJI's internal
 *    image processing (cropping, stabilization, ISP) and the difficulty
 *    of accurately calibrating a very narrow field of view.
 *  - Distortion coefficients above 5× are not repeatable between
 *    calibrations and are therefore considered unreliable.
 *
 * The generated CameraInfo therefore uses a hybrid model:
 *
 *  - fx/fy:
 *      • interpolated between the 2×, 5× and 10× calibrations
 *      • scaled linearly beyond 10× using the current optical zoom
 *
 *  - cx/cy:
 *      • interpolated between 2× and 5×
 *      • fixed to the 5× calibration above 5×
 *
 *  - distortion:
 *      • interpolated between 2× and 5×
 *      • fixed to the 5× calibration above 5×
 *
 * This approach prioritizes parameters that proved repeatable during
 * calibration while avoiding the unstable high-zoom estimates, resulting
 * in a more robust camera model over the full 2×–50× zoom range.
 */
sensor_msgs::msg::CameraInfo
LiveviewModule::get_camera_info_zoom(double zoom_factor)
{
  sensor_msgs::msg::CameraInfo cam2 = camera_infos_[CAMERA_INFO_H20T_ZOOM_2X];
  sensor_msgs::msg::CameraInfo cam5 = camera_infos_[CAMERA_INFO_H20T_ZOOM_5X];
  sensor_msgs::msg::CameraInfo cam10 = camera_infos_[CAMERA_INFO_H20T_ZOOM_10X];

  sensor_msgs::msg::CameraInfo out = cam2;

  //-----------------------------
  // focal length
  //-----------------------------

  if (zoom_factor <= 5.0)
  {
    double t = (zoom_factor - 2.0) / 3.0;

    out.k[0] = lerp(cam2.k[0], cam5.k[0], t);
    out.k[4] = lerp(cam2.k[4], cam5.k[4], t);
  }
  else if (zoom_factor <= 10.0)
  {
    double t = (zoom_factor - 5.0) / 5.0;

    out.k[0] = lerp(cam5.k[0], cam10.k[0], t);
    out.k[4] = lerp(cam5.k[4], cam10.k[4], t);
  }
  else
  {
    double s = zoom_factor / 10.0;

    out.k[0] = cam10.k[0] * s;
    out.k[4] = cam10.k[4] * s;
  }

  //-----------------------------
  // principal point
  //-----------------------------

  if (zoom_factor <= 5.0)
  {
    double t = (zoom_factor - 2.0) / 3.0;

    out.k[2] = lerp(cam2.k[2], cam5.k[2], t);
    out.k[5] = lerp(cam2.k[5], cam5.k[5], t);
  }
  else
  {
    out.k[2] = cam5.k[2];
    out.k[5] = cam5.k[5];
  }

  //-----------------------------
  // distortion
  //-----------------------------

  if (zoom_factor <= 5.0)
  {
    double t = (zoom_factor - 2.0) / 3.0;

    for (size_t i = 0; i < 5; i++) out.d[i] = lerp(cam2.d[i], cam5.d[i], t);
  }
  else
  {
    out.d = cam5.d;
  }

  //-----------------------------
  // projection
  //-----------------------------

  out.p.fill(0);

  out.p[0] = out.k[0];
  out.p[2] = out.k[2];

  out.p[5] = out.k[4];
  out.p[6] = out.k[5];

  out.p[10] = 1.0;

  return out;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring LiveviewModule");
  main_camera_stream_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "psdk_ros2/main_camera_stream/compressed", rclcpp::SensorDataQoS());
  fpv_camera_stream_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "psdk_ros2/fpv_camera_stream", rclcpp::SensorDataQoS());
  camera_setup_streaming_service_ = create_service<CameraSetupStreaming>(
      "psdk_ros2/camera_setup_streaming",
      std::bind(&LiveviewModule::camera_setup_streaming_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "psdk_ros2/main_camera_stream/camera_info", rclcpp::SensorDataQoS());

  E_DjiCameraType camera_type =
      psdk_ros2::global_camera_ptr_->get_attached_camera_type();

  if (camera_type == DJI_CAMERA_TYPE_H20T)
    stream_state_.camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE;

  auto camera_calib_folder =
      ament_index_cpp::get_package_share_directory("lrs_m300") + "/configs";
  load_camera_info_files(camera_calib_folder);

  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating LiveviewModule");
  main_camera_stream_pub_->on_activate();
  fpv_camera_stream_pub_->on_activate();
  camera_info_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating LiveviewModule");
  main_camera_stream_pub_->on_deactivate();
  fpv_camera_stream_pub_->on_deactivate();
  camera_info_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up LiveviewModule");
  camera_setup_streaming_service_.reset();
  main_camera_stream_pub_.reset();
  fpv_camera_stream_pub_.reset();
  camera_info_pub_.reset();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down LiveviewModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_liveview_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

// void custom_log_callback(void *ptr, int level, const char *fmt, va_list vl) {
//    if (strstr(fmt, "SEI type 1 size") != NULL)
//        return; // skip this message
//
//    vfprintf(stderr, fmt, vl);
//}
//
// av_log_set_callback(custom_log_callback);

bool
LiveviewModule::init()
{
  // suppress warnings like: SEI type 1 size 131 truncated at 106
  // see above for a different solution
  av_log_set_level(AV_LOG_QUIET);

  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Liveview module is already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating liveview module");

  declare_parameter<int>("main_camera_width", -1);
  get_parameter("main_camera_width", main_camera_image_width);
  declare_parameter<int>("main_camera_height", -1);
  get_parameter("main_camera_height", main_camera_image_height);
  declare_parameter<int>("main_camera_jpeg_quality", 80);
  get_parameter("main_camera_jpeg_quality", main_camera_jpeg_quality);
  declare_parameter<int>("image_time_offset_ms", 0);
  get_parameter("image_time_offset_ms", image_time_offset_ms);

  parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &LiveviewModule::parametersCallback, this, std::placeholders::_1));

  T_DjiReturnCode return_code = DjiLiveview_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  /* Start decoders*/
  stream_decoder_ = {
      {DJI_LIVEVIEW_CAMERA_POSITION_FPV, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
  };
  decode_stream_ = true;
  //  payload_index_ = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
  is_module_initialized_ = true;
  return true;
}

rcl_interfaces::msg::SetParametersResult
LiveviewModule::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == "main_camera_jpeg_quality")
    {
      int quality = parameter.as_int();

      if (quality < 1 || quality > 100)
      {
        result.successful = false;
        result.reason = "jpeg_quality must be between 1 and 100";
        return result;
      }

      set_main_camera_jpeg_quality(quality);


    }
    if (parameter.get_name() == "image_time_offset_ms")
    {
      int quality = parameter.as_int();

      image_time_offset_ms = quality;

      RCLCPP_INFO(get_logger(), "Image time offset set to %d",
                  image_time_offset_ms);
    }
  }

  return result;
}

bool
LiveviewModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing liveview module");
  T_DjiReturnCode return_code = DjiLiveview_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buffer,
                                   uint32_t buffer_length)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);

  if (global_liveview_ptr_->decode_stream_)
  {
    return global_liveview_ptr_->LiveviewConvertH264ToRgbCallback(
        position, buffer, buffer_length);
  }

  if (global_liveview_ptr_->stream_state_.payload_index ==
      DJI_LIVEVIEW_CAMERA_POSITION_FPV)
  {
    return global_liveview_ptr_->publish_fpv_camera_images(buffer,
                                                           buffer_length);
  }
  return global_liveview_ptr_->publish_main_camera_images(buffer,
                                                          buffer_length);
}

void
LiveviewModule::LiveviewConvertH264ToRgbCallback(
    E_DjiLiveViewCameraPosition position, const uint8_t *buffer,
    uint32_t buffer_length)
{
  auto decoder = stream_decoder_.find(position);
  if ((decoder != stream_decoder_.end()) && decoder->second)
  {
    decoder->second->decodeBuffer(buffer, buffer_length);
  }
}

void
c_publish_main_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_main_camera_images(img, user_data);
}

void
c_publish_fpv_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_fpv_camera_images(img, user_data);
}

bool
LiveviewModule::camera_setup_streaming(bool start, int payload_index,
                                       int camera_source, bool decoded_output)
{
  E_DjiLiveViewCameraPosition dji_payload_index = stream_state_.payload_index;
  if (payload_index != -1)
  {
    dji_payload_index = static_cast<E_DjiLiveViewCameraPosition>(payload_index);
    //    payload_index_ = p_index;
  }

  E_DjiLiveViewCameraSource dji_camera_source = stream_state_.camera_source;

  if (camera_source != -1)
  {
    dji_camera_source = static_cast<E_DjiLiveViewCameraSource>(camera_source);
    //    selected_camera_source_ = cam_source;
  }
  decode_stream_ = decoded_output;

  RCLCPP_INFO(get_logger(),
              "Setting up camera streaming for payload index %d, camera source "
              "%d, decoded %d, starting: %d",
              dji_payload_index, dji_camera_source, decode_stream_, start);

  if (start)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming...");

    if (dji_payload_index == DJI_LIVEVIEW_CAMERA_POSITION_NO_1)
    {
      static char main_camera_name[] = "MAIN_CAMERA";

      return start_camera_stream(&c_publish_main_streaming_callback,
                                 main_camera_name, dji_payload_index,
                                 dji_camera_source);
    }

    if (dji_payload_index == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
    {
      static char fpv_camera_name[] = "FPV_CAMERA";

      return start_camera_stream(&c_publish_fpv_streaming_callback,
                                 fpv_camera_name, dji_payload_index,
                                 dji_camera_source);
    }

    RCLCPP_ERROR(get_logger(), "Unsupported payload index %d",
                 dji_payload_index);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Stopping streaming... payload: %d camera: %d",
              dji_payload_index, dji_camera_source);

  return stop_main_camera_stream(dji_payload_index, dji_camera_source);
}

bool
LiveviewModule::is_streaming() const
{
  return stream_state_.streaming;
}

int
LiveviewModule::get_main_camera_jpeg_quality() const
{
  return main_camera_jpeg_quality;
}

void
LiveviewModule::set_main_camera_jpeg_quality(const int &jpeg_quality)
{
  RCLCPP_INFO(get_logger(), "Main camera JPEG quality changed to %d",
              main_camera_jpeg_quality);

  main_camera_jpeg_quality = jpeg_quality;
}

std::string
LiveviewModule::get_camera_lens_name() const
{
  switch (stream_state_.camera_source)
  {
    case DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT:
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE:
      return "wide";
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM:
      return "zoom";
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR:
      return "ir";
  }
  return "unknown";
}

int
LiveviewModule::get_camera_source_index() const
{
  switch (stream_state_.camera_source)
  {
    case DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT:
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_WIDE:
      return 0;
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_ZOOM:
      return 1;
    case DJI_LIVEVIEW_CAMERA_SOURCE_H20T_IR:
      return 2;
  }
  return -1;
}

void
LiveviewModule::camera_setup_streaming_cb(
    const std::shared_ptr<CameraSetupStreaming::Request> request,
    const std::shared_ptr<CameraSetupStreaming::Response> response)
{
  response->success =
      camera_setup_streaming(request->start_stop, request->payload_index,
                             request->camera_source, request->decoded_output);
}

bool
LiveviewModule::start_camera_stream(CameraImageCallback callback,
                                    void *user_data,
                                    E_DjiLiveViewCameraPosition payload_index,
                                    E_DjiLiveViewCameraSource camera_source)
{
  //  RCLCPP_INFO(rclcpp::get_logger("liveview"), "start_camera_stream");
  if (decode_stream_)
  {
    //    RCLCPP_INFO(rclcpp::get_logger("liveview"),
    //                "start_camera_stream: decode_stream_");
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->init();
      decoder->second->registerCallback(callback, user_data);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to set-up the decoder");
      return false;
    }
  }

  //  RCLCPP_INFO(rclcpp::get_logger("liveview"), "start_camera_stream: %d %d",
  //              payload_index, camera_source);
  T_DjiReturnCode return_code = DjiLiveview_StartH264Stream(
      payload_index, camera_source, c_LiveviewConvertH264ToRgbCallback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to start camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully started the camera streaming.");
    stream_state_.streaming = true;
    stream_state_.payload_index = payload_index;
    stream_state_.camera_source = camera_source;
    //    payload_index_ = payload_index;
    return true;
  }
}

bool
LiveviewModule::stop_main_camera_stream(
    const E_DjiLiveViewCameraPosition payload_index,
    const E_DjiLiveViewCameraSource camera_source)
{
  T_DjiReturnCode return_code =
      DjiLiveview_StopH264Stream(payload_index, camera_source);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to stop camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->cleanup();
    }
    RCLCPP_INFO(get_logger(), "Successfully stopped camera streaming.");
    stream_state_.streaming = false;

    return true;
  }
}

void
LiveviewModule::publish_main_camera_images(const uint8_t *buffer,
                                           uint32_t buffer_length)
{
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = get_optical_frame_id();
  //  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_fpv_camera_images(const uint8_t *buffer,
                                          uint32_t buffer_length)
{
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  fpv_camera_stream_pub_->publish(std::move(img));
}

CameraRGBImage
LiveviewModule::scaleDownByHalf(const CameraRGBImage &input)
{
  CameraRGBImage output;

  double scale_factor = 2.0;

  // New dimensions
  output.width = input.width / scale_factor;
  output.height = input.height / scale_factor;
  output.rawData.resize(output.width * output.height * 3);

  for (int y = 0; y < output.height; ++y)
  {
    for (int x = 0; x < output.width; ++x)
    {
      // Corresponding pixel in the input image (nearest neighbor)
      int srcX = x * scale_factor;
      int srcY = y * scale_factor;

      int inputIndex = (srcY * input.width + srcX) * 3;
      int outputIndex = (y * output.width + x) * 3;

      output.rawData[outputIndex + 0] = input.rawData[inputIndex + 0];  // R
      output.rawData[outputIndex + 1] = input.rawData[inputIndex + 1];  // G
      output.rawData[outputIndex + 2] = input.rawData[inputIndex + 2];  // B
    }
  }

  return output;
}

void
LiveviewModule::publish_main_camera_images(CameraRGBImage rgb_img,
                                           void *user_data)
{
  //  auto rgb_img = scaleDownByHalf(rgb_img_in);

  auto now = this->now();

  // ---- Create cv::Mat directly (avoid extra copies from cv_bridge)
  cv::Mat img(rgb_img.height, rgb_img.width, CV_8UC3, rgb_img.rawData.data());

  // ---- Resize ----
  int cols = img.cols;
  int rows = img.rows;

  if ((main_camera_image_width > 0) && (main_camera_image_height > 0))
  {
    cols = main_camera_image_width;
    rows = main_camera_image_height;
  }

  cv::Mat img_bgr;
  cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);

  cv::Mat outimg;
  cv::resize(img_bgr, outimg, cv::Size(cols, rows));

  // ---- Compress with lower JPEG quality ----
  std::vector<uchar> buffer;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                             main_camera_jpeg_quality};

  cv::imencode(".jpg", outimg, buffer, params);

  rclcpp::Duration offset =
      rclcpp::Duration::from_nanoseconds(image_time_offset_ms * 1e6);

  // ---- Build CompressedImage
  sensor_msgs::msg::CompressedImage msg;
  auto stamp_now = this->get_clock()->now();
  msg.header.stamp = stamp_now - offset;
  std::string ns = get_namespace();
  std::string unit = ns.substr(1);
  msg.header.frame_id = unit + "/camera0/image_frame";
  msg.format = "jpeg";
  msg.data = std::move(buffer);

  // ---- Publish ----
  main_camera_stream_pub_->publish(msg);

  E_DjiCameraType camera_type =
      psdk_ros2::global_camera_ptr_->get_attached_camera_type();

  float zoom_factor = psdk_ros2::global_camera_ptr_->get_zoom_factor();

  auto camera_info = get_camera_info(camera_type, stream_state_.camera_source,
                                     cols, rows, zoom_factor);
  camera_info.header = msg.header;
//  camera_info.header.stamp = stamp_now-offset;
  camera_info_pub_->publish(camera_info);
#if 0
  auto t1 = this->now();
  (void)user_data;
  // auto img = std::make_unique<sensor_msgs::msg::Image>();
  auto img = sensor_msgs::msg::Image();
  img.height = rgb_img.height;
  img.width = rgb_img.width;
  img.step = rgb_img.width * 3;
  img.encoding = "rgb8";
  img.data = rgb_img.rawData;

//  RCLCPP_INFO_STREAM(get_logger(), "publish_main_camera_images " <<rgb_img.width << " " << rgb_img.height);
  cv_bridge::CvImagePtr cv_ptr;  
  cv::Mat outimg;
  try {    
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image;
    int cols = img.cols/2;
    int rows = img.rows/2;
    get_parameter("main_camera_width", main_camera_width);
    get_parameter("main_camera_height", main_camera_height);
    if ((main_camera_width > 0) && (main_camera_height > 0)) {
      cols = main_camera_width;
      rows = main_camera_height;
    }
    // RCLCPP_ERROR(get_logger(), "cv_bridge resize: %d - %d", cols, rows);    
    //cv::resize(img, outimg, cv::Size(cols, rows), 0, 0, CV_INTER_LINEAR);
    cv::resize(img, outimg, cv::Size(cols, rows));
  }

  catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  double t_diff = (this->now() - t1).seconds()*1000.0;
  RCLCPP_INFO_STREAM(get_logger(), "make img dt " << t_diff);

  t1 = this->now();

  cv_bridge::CvImage cv_image(cv_ptr->header, cv_ptr->encoding, outimg);

  t_diff = (this->now() - t1).seconds()*1000.0;
  RCLCPP_INFO_STREAM(get_logger(), "cv img " << t_diff);

  t1 = this->now();



  auto cimg = cv_image.toCompressedImageMsg();
  t_diff = (this->now() - t1).seconds()*1000.0;
  RCLCPP_INFO_STREAM(get_logger(), "compress " << t_diff);

  cimg->header.stamp = this->get_clock()->now();
  // cimg->header.frame_id = get_optical_frame_id();
  std::string ns = get_namespace();
  std::string unit = ns.substr(1);  
  cimg->header.frame_id = unit + "/camera0/image_frame";
  //img.header.stamp = this->get_clock()->now();
  //img.header.frame_id = get_optical_frame_id();
  //main_camera_stream_pub_->publish(std::move(img));
  // main_camera_stream_pub_->publish(*(cv_ptr->toCompressedImageMsg()));


  t1 = this->now();
  main_camera_stream_pub_->publish(*cimg);


  t_diff = (this->now() - t1).seconds()*1000.0;
  RCLCPP_INFO_STREAM(get_logger(), "publish " << t_diff);

#endif

  double freq = updateFrequency(timestamps_, this->get_clock()->now(), 5.0);

  float time_s = 10;
  //  RCLCPP_INFO(get_logger(), "Creating LiveviewModule");
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), time_s * 1000,
                       "Publishing main camera image (%.0fs),  in: (%d,%d) "
                       "out: (%d,%d) freq (5s): %.2f",
                       time_s, rgb_img.width, rgb_img.height, cols, rows, freq);

  //  auto t_diff = (this->now() - now).seconds() * 1000.0;
  //  RCLCPP_INFO_STREAM(get_logger(), "total " << t_diff);
}

double
LiveviewModule::updateFrequency(std::deque<rclcpp::Time> &timestamps,
                                const rclcpp::Time &now, double window_sec)
{
  // Add new timestamp
  timestamps.push_back(now);

  // Remove old ones
  while (!timestamps.empty() &&
         (now - timestamps.front()).seconds() > window_sec)
  {
    timestamps.pop_front();
  }

  // Compute frequency
  if (timestamps.size() < 2) return 0.0;

  double duration = (timestamps.back() - timestamps.front()).seconds();

  if (duration <= 0.0) return 0.0;

  return (timestamps.size() - 1) / duration;
}

void
LiveviewModule::publish_fpv_camera_images(CameraRGBImage rgb_img,
                                          void *user_data)
{
  (void)user_data;
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->height = rgb_img.height;
  img->width = rgb_img.width;
  img->step = rgb_img.width * 3;
  img->encoding = "rgb8";
  img->data = rgb_img.rawData;

  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  fpv_camera_stream_pub_->publish(std::move(img));
}

std::string
LiveviewModule::get_optical_frame_id()
{
  for (auto &it : psdk_utils::camera_source_str)
  {
    if (it.first == stream_state_.camera_source)
    {
      return it.second;
    }
  }
  return "";
}

}  // namespace psdk_ros2
