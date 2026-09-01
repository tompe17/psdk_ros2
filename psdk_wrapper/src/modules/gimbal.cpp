/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file gimbal.cpp
 *
 * @brief
 *
 * @author Lidia de la Torre Vazquez
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/modules/gimbal.hpp"

#include "psdk_wrapper/modules/telemetry.hpp"
namespace psdk_ros2
{

GimbalModule::GimbalModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating GimbalModule");
}

GimbalModule::~GimbalModule()
{
  RCLCPP_INFO(get_logger(), "Destroying GimbalModule");
}

GimbalModule::CallbackReturn
GimbalModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring GimbalModule");
  gimbal_rotation_sub_ =
      create_subscription<psdk_interfaces::msg::GimbalRotation>(
          "psdk_ros2/gimbal_rotation", 10,
          std::bind(&GimbalModule::gimbal_rotation_cb, this,
                    std::placeholders::_1));
  gimbal_set_mode_service_ = create_service<GimbalSetMode>(
      "psdk_ros2/gimbal_set_mode",
      std::bind(&GimbalModule::gimbal_set_mode_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  gimbal_reset_service_ = create_service<GimbalReset>(
      "psdk_ros2/gimbal_reset",
      std::bind(&GimbalModule::gimbal_reset_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  return CallbackReturn::SUCCESS;
}
GimbalModule::CallbackReturn
GimbalModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating GimbalModule");
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating GimbalModule");
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up GimbalModule");
  gimbal_set_mode_service_.reset();
  gimbal_reset_service_.reset();
  gimbal_rotation_sub_.reset();
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down GimbalModule");
  return CallbackReturn::SUCCESS;
}

bool
GimbalModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_INFO(get_logger(), "Gimbal manager already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating gimbal manager");
  T_DjiReturnCode return_code = DjiGimbalManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize gimbal manager. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = true;
  return true;
}

bool
GimbalModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing gimbal manager");
  T_DjiReturnCode return_code = DjiGimbalManager_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize gimbal manager. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

bool
GimbalModule::set_mode_follow()
{
  // auto telem = global_telemetry_ptr_;
  // telem->body_yaw_raw_at_reset_rad_ -= telem->offset_due_to_yaw;

  return set_gimbal_mode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                         DJI_GIMBAL_MODE_YAW_FOLLOW);
}

bool
GimbalModule::set_mode_free()
{
  return set_gimbal_mode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                         DJI_GIMBAL_MODE_FREE);
}

void
GimbalModule::gimbal_set_mode_cb(
    const std::shared_ptr<GimbalSetMode::Request> request,
    const std::shared_ptr<GimbalSetMode::Response> response)
{
  response->success =
      set_gimbal_mode(static_cast<E_DjiMountPosition>(request->payload_index),
                      static_cast<E_DjiGimbalMode>(request->gimbal_mode));
#if 0
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiGimbalMode gimbal_mode =
      static_cast<E_DjiGimbalMode>(request->gimbal_mode);
  return_code = DjiGimbalManager_SetMode(index, gimbal_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Setting gimbal mode failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Setting gimbal mode successfully to %d",
                request->gimbal_mode);
    gimbal_mode_ = gimbal_mode;
    response->success = true;
    return;
  }
#endif
}

bool
GimbalModule::set_gimbal_mode(E_DjiMountPosition index,
                              E_DjiGimbalMode gimbal_mode)
{
  T_DjiReturnCode return_code = DjiGimbalManager_SetMode(index, gimbal_mode);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Setting gimbal mode failed, error code: %ld",
                 return_code);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Setting gimbal mode successfully to %d",
              static_cast<int>(gimbal_mode));

  gimbal_mode_ = gimbal_mode;

  return true;
}

bool
GimbalModule::reset_gimbal(E_DjiMountPosition index,
                           E_DjiGimbalResetMode reset_mode)
{
  T_DjiReturnCode return_code = DjiGimbalManager_Reset(index, reset_mode);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Reset gimbal failed, error code: %ld",
                 return_code);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Gimbal resetting...");

  global_telemetry_ptr_->current_state_.gimbal_angle_history.clear();

  const bool yaw_reset =
      reset_mode == DJI_GIMBAL_RESET_MODE_YAW ||
      reset_mode == DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW ||
      reset_mode == DJI_GIMBAL_RESET_MODE_YAW_ONLY ||
      reset_mode == DJI_GIMBAL_RESET_MODE_PITCH_DOWNWARD_UPWARD_AND_YAW;

  while (!global_telemetry_ptr_->current_state_.gimbal_angle_history.stable())
  {
    RCLCPP_INFO(get_logger(), "Waiting for gimbal to stabilize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (yaw_reset)
  {
    global_telemetry_ptr_->save_body_gimbal_offset();
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Gimbal yaw (pan) not included in gimbal reset.");
  }

  RCLCPP_INFO(get_logger(), "Gimbal reset complete.");

  return true;
}

void
GimbalModule::gimbal_reset_cb(
    const std::shared_ptr<GimbalReset::Request> request,
    const std::shared_ptr<GimbalReset::Response> response)
{
  response->success =
      reset_gimbal(static_cast<E_DjiMountPosition>(request->payload_index),
                   static_cast<E_DjiGimbalResetMode>(request->reset_mode));
}

bool
GimbalModule::rotate_gimbal(E_DjiMountPosition index,
                            E_DjiGimbalRotationMode rotation_mode, double roll,
                            double pitch, double yaw, double time)
{
  if (!is_module_initialized_)
  {
    return false;
  }


  T_DjiGimbalManagerRotation rotation_deg;

  rotation_deg.rotationMode = rotation_mode;

  // DJI PSDK expects FRD. Convert from ROS FLU.
  rotation_deg.pitch = psdk_utils::rad_to_deg(-pitch);
  rotation_deg.roll = psdk_utils::rad_to_deg(roll);

  if (rotation_mode == DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE)
  {
    rotation_deg.yaw = psdk_utils::rad_to_deg(-yaw);
  }
  else
  {
    //original
    //rotation_deg.yaw = psdk_utils::rad_to_deg(psdk_utils::SHIFT_N2E - yaw);


    // it's the inverse of calculating angles telemetry
    double z = yaw;

    auto t = global_telemetry_ptr_;
    if (t->params_.sim && global_gimbal_ptr_->gimbal_mode_is_follow())
    {
      auto offset_due_to_yaw =
          t->body_yaw_raw_at_reset_rad_ - t->get_body_yaw_raw_rad();
      rotation_deg.yaw += psdk_utils::rad_to_deg(offset_due_to_yaw);

      z -= offset_due_to_yaw;
      // z -= global_telemetry_ptr_->offset_due_to_yaw;
    }
    rotation_deg.yaw = psdk_utils::rad_to_deg(psdk_utils::SHIFT_N2E - z) +
                       t->body_gimbal_offset_raw_deg_;
  }

  rotation_deg.time = time;

  // if (!set_gimbal_mode(index, DJI_GIMBAL_MODE_FREE))
  // {
  //   return false;
  // }
  RCLCPP_INFO(
      get_logger(),
      "rotate_gimbal RPY = (%.1f, %.1f, %.1f) mode= %d",
      rotation_deg.roll, rotation_deg.pitch, rotation_deg.yaw, rotation_mode);


  T_DjiReturnCode return_code = DjiGimbalManager_Rotate(index, rotation_deg);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Target gimbal RPY = (%.1f, %.1f, %.1f) failed, error code: %ld",
        rotation_deg.pitch, rotation_deg.roll, rotation_deg.yaw, return_code);
    return false;
  }

  return true;
}

void
GimbalModule::gimbal_rotation_cb(
    const psdk_interfaces::msg::GimbalRotation::SharedPtr msg)
{
#if 0
  (void)msg;
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(msg->payload_index);
  T_DjiGimbalManagerRotation rotation_deg;
  rotation_deg.rotationMode =
      static_cast<E_DjiGimbalRotationMode>(msg->rotation_mode);
  /** DJI PSDK seems to take roll, pitch and yaw when used in incremental mode
   * wrt. a FRD frame. Here this is converted to FLU*/
  rotation_deg.pitch = psdk_ros2::psdk_utils::rad_to_deg(-msg->pitch);
  rotation_deg.roll = psdk_ros2::psdk_utils::rad_to_deg(msg->roll);
  if (msg->rotation_mode == DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE)
  {
    rotation_deg.yaw = psdk_ros2::psdk_utils::rad_to_deg(-msg->yaw);
  }
  else
  {
    rotation_deg.yaw =
        psdk_ros2::psdk_utils::rad_to_deg(psdk_utils::SHIFT_N2E - msg->yaw);
    // pioru: test if needed
    rotation_deg.yaw += global_telemetry_ptr_->body_gimbal_offset_raw_deg_;
  }

  rotation_deg.time = msg->time;

  // return_code = DjiGimbalManager_SetMode(index, DJI_GIMBAL_MODE_YAW_FOLLOW);
  return_code = DjiGimbalManager_SetMode(index, DJI_GIMBAL_MODE_FREE);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Set gimbal mode failed, error code: %ld",
                 return_code);
    return;
  }

  return_code = DjiGimbalManager_Rotate(index, rotation_deg);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(
        get_logger(),
        "Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: %ld",
        rotation_deg.pitch, rotation_deg.roll, rotation_deg.yaw, return_code);
    return;
  }
#endif
  rotate_gimbal(static_cast<E_DjiMountPosition>(msg->payload_index),
                static_cast<E_DjiGimbalRotationMode>(msg->rotation_mode),
                msg->roll, msg->pitch, msg->yaw, msg->time);
}

}  // namespace psdk_ros2
