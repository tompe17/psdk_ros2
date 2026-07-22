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

  // declare_parameter("location", "granso");

  // get_parameter("location", location_);

  // RCLCPP_INFO(get_logger(), "Location: %s", location_.c_str());
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

  ct_ = new CoordTrans(location_);
  double lon, lat;
  ct_->world_to_wgs84(0.0, 0.0, 0.0, lon, lat, world_origin_elevation_);
  RCLCPP_INFO(get_logger(), "World origin elevation: %g",
              world_origin_elevation_);

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

// CoordModule::set_takeoff_info(const lrs_m300_msgs::msg::TakeoffInfo & msg)
// {
// takeoff_height_above_ellipsoid = msg.height_above_ellipsoid;
// takeoff_gps_altitude = msg.gps_altitude;
// have_takeoff_info = true;
// }

void
CoordModule::wgs84_to_world(double lon, double lat, double alt, double &x,
                            double &y, double &z) const
{
  ct_->wgs84_to_world(lon, lat, alt, x, y, z);
}


#if 0
// pioru: unused - it's in telemetry
void
CoordModule::publish_tf(
    const sensor_msgs::msg::NavSatFix &gps_position_fused_msg,
    const sensor_msgs::msg::NavSatFix &home_point_msg,
    const std_msgs::msg::Float32 &altitude_sl_fused_msg,
    const geometry_msgs::msg::QuaternionStamped &current_attitude_msg)
{
  const double height_above_takeoff =
      altitude_sl_fused_msg.data - home_point_msg.altitude;

  double cx, cy, cz;
  double alt = height_above_takeoff + world_origin_elevation_;
  ct_->wgs84_to_world(gps_position_fused_msg.longitude,
                      gps_position_fused_msg.latitude, alt, cx, cy, cz);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = gps_position_fused_msg.header.stamp;
  t.header.frame_id = map_frame_;                           // "world"
  t.child_frame_id = tf_frame_prefix_ + "/" + body_frame_;  // unit + "/body";

  t.transform.translation.x = cx;
  t.transform.translation.y = cy;
  t.transform.translation.z = cz;
  t.transform.rotation = current_attitude_msg.quaternion;
  tf_broadcaster_->sendTransform(t);

  t.child_frame_id =
      tf_frame_prefix_ + "/" + horbody_frame_;  // unit + "/body_hor";

  tf2::Quaternion tf_q;
  tf2::fromMsg(current_attitude_msg.quaternion, tf_q);
  tf2::Quaternion tf_q_flat_yaw;
  tf_q_flat_yaw.setRPY(0.0, 0.0, tf2::getYaw(tf_q));
  t.transform.rotation = tf2::toMsg(tf_q_flat_yaw);
  tf_broadcaster_->sendTransform(t);
}
#endif
}  // namespace psdk_ros2