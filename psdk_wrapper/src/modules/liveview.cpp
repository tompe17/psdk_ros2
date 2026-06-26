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
}

LiveviewModule::~LiveviewModule()
{
  RCLCPP_INFO(get_logger(), "Destroying LiveviewModule");
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
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating LiveviewModule");
  main_camera_stream_pub_->on_activate();
  fpv_camera_stream_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating LiveviewModule");
  main_camera_stream_pub_->on_deactivate();
  fpv_camera_stream_pub_->on_deactivate();
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

  declare_parameter<int>("main_camera_width", -1);
  get_parameter("main_camera_width", main_camera_width);
  declare_parameter<int>("main_camera_height", -1);
  get_parameter("main_camera_height", main_camera_height);
  declare_parameter<int>("main_camera_jpeg_quality", 80);
  get_parameter("main_camera_jpeg_quality", main_camera_jpeg_quality);

  RCLCPP_INFO(get_logger(), "Initiating liveview module");
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
  payload_index_ = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
  is_module_initialized_ = true;
  return true;
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

  if (global_liveview_ptr_->payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
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
  if (payload_index != -1)
  {
    E_DjiLiveViewCameraPosition p_index =
        static_cast<E_DjiLiveViewCameraPosition>(payload_index);
    payload_index_ = p_index;
  }

  if (camera_source != -1)
  {
    E_DjiLiveViewCameraSource cam_source =
        static_cast<E_DjiLiveViewCameraSource>(camera_source);
    selected_camera_source_ = cam_source;
  }
  decode_stream_ = decoded_output;

  RCLCPP_INFO(get_logger(),
              "Setting up camera streaming for payload index %d, camera source "
              "%d, decoded %d, starting: %d",
              payload_index_, selected_camera_source_, decode_stream_, start);

  if (start)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming...");

    if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_NO_1)
    {
      static char main_camera_name[] = "MAIN_CAMERA";

      return start_camera_stream(&c_publish_main_streaming_callback,
                                 main_camera_name, payload_index_,
                                 selected_camera_source_);
    }

    if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
    {
      static char fpv_camera_name[] = "FPV_CAMERA";

      return start_camera_stream(&c_publish_fpv_streaming_callback,
                                 fpv_camera_name, payload_index_,
                                 selected_camera_source_);
    }

    RCLCPP_ERROR(get_logger(), "Unsupported payload index %d", payload_index_);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Stopping streaming... payload: %d camera: %d", payload_index_, selected_camera_source_);

  return stop_main_camera_stream(payload_index_, selected_camera_source_);
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

#if 0

void
LiveviewModule::camera_setup_streaming_cb(
    const std::shared_ptr<CameraSetupStreaming::Request> request,
    const std::shared_ptr<CameraSetupStreaming::Response> response)
{
  selected_camera_source_ =
      static_cast<E_DjiLiveViewCameraSource>(request->camera_source);
  decode_stream_ = request->decoded_output;
  payload_index_ =
      static_cast<E_DjiLiveViewCameraPosition>(request->payload_index);

  RCLCPP_INFO(get_logger(),
              "Setting up camera streaming for payload index %d and camera "
              "source %d. Output decoded: %d",
              payload_index_, selected_camera_source_, decode_stream_);

  if (request->start_stop)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming... fpvvam: %d",
                DJI_LIVEVIEW_CAMERA_POSITION_FPV);
    bool streaming_result;
    if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_NO_1)
    {
      RCLCPP_INFO(get_logger(), "MAIN_CAMERA...");
      char main_camera_name[] = "MAIN_CAMERA";
      streaming_result = start_camera_stream(&c_publish_main_streaming_callback,
                                             &main_camera_name, payload_index_,
                                             selected_camera_source_);
    }
    else if (payload_index_ == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
    {
      RCLCPP_INFO(get_logger(), "FPV_CAMERA...");
      char fpv_camera_name[] = "FPV_CAMERA";
      streaming_result = start_camera_stream(&c_publish_fpv_streaming_callback,
                                             &fpv_camera_name, payload_index_,
                                             selected_camera_source_);
    }

    if (streaming_result)
    {
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Stopping camera streaming...");
    if (stop_main_camera_stream(payload_index_, selected_camera_source_))
    {
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
}

#endif

bool
LiveviewModule::start_camera_stream(CameraImageCallback callback,
                                    void *user_data,
                                    E_DjiLiveViewCameraPosition payload_index,
                                    E_DjiLiveViewCameraSource camera_source)
{
  RCLCPP_INFO(rclcpp::get_logger("liveview"), "start_camera_stream");
  if (decode_stream_)
  {
    RCLCPP_INFO(rclcpp::get_logger("liveview"),
                "start_camera_stream: decode_stream_");
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
  RCLCPP_INFO(rclcpp::get_logger("liveview"), "start_camera_stream: %d %d",
              payload_index, camera_source);
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

  get_parameter("main_camera_width", main_camera_width);
  get_parameter("main_camera_height", main_camera_height);
  get_parameter("main_camera_jpeg_quality", main_camera_jpeg_quality);

  //  RCLCPP_INFO_STREAM(get_logger(),
  //                     "main_camera_jpeg_quality " <<
  //                     main_camera_jpeg_quality);
  //
  if ((main_camera_width > 0) && (main_camera_height > 0))
  {
    cols = main_camera_width;
    rows = main_camera_height;
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

  // ---- Build CompressedImage
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = this->get_clock()->now();
  std::string ns = get_namespace();
  std::string unit = ns.substr(1);
  msg.header.frame_id = unit + "/camera0/image_frame";
  msg.format = "jpeg";
  msg.data = std::move(buffer);

  // ---- Publish ----
  main_camera_stream_pub_->publish(msg);
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
    if (it.first == selected_camera_source_)
    {
      return it.second;
    }
  }
}

}  // namespace psdk_ros2
