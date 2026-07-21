/*
 * Copyright (C) 2024
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0.
 */

/**
 * @file coord.hpp
 *
 * @brief Header file for the CoordModule class
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_COORD_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_COORD_HPP_


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "coordtrans.h"

namespace psdk_ros2
{

class CoordModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CoordModule(const std::string &name);

  ~CoordModule();

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  bool init();
  bool deinit();

  void wgs84_to_world(double lon, double lat, double alt, double &x, double &y,
                      double &z) const;
  double get_world_origin_elevation() const
  {
    return world_origin_elevation_;
  }
  std::string location_;

 private:
  CoordTrans *ct_;
  double world_origin_elevation_;
};

extern std::shared_ptr<CoordModule> global_coord_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_COORD_HPP_