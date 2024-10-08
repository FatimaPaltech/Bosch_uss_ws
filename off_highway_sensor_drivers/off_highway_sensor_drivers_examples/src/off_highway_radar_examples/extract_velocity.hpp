// Copyright 2024 Robert Bosch GmbH and its subsidiaries
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace off_highway_sensor_drivers_examples
{

class ExtractVelocity : public rclcpp::Node
{
public:
  /**
   * \brief Construct a new ExtractVelocity object.
   */
  explicit ExtractVelocity(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * \brief Extract velocity message.
   *
   * \param msg Received message data
   */
  void callback_input(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
};
}  // namespace off_highway_sensor_drivers_examples
