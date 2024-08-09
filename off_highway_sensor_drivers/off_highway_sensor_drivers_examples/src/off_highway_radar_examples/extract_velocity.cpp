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

#include <memory>
#include <utility>

#include "extract_velocity.hpp"

namespace off_highway_sensor_drivers_examples
{
ExtractVelocity::ExtractVelocity(const rclcpp::NodeOptions & options)
: rclcpp::Node("extract_velocity", options)
{
  input_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    10,
    std::bind(&ExtractVelocity::callback_input, this, std::placeholders::_1)
  );
  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
}

void ExtractVelocity::callback_input(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  auto twist_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_stamped_msg->header = msg->header;
  twist_stamped_msg->twist = msg->twist.twist;
  output_pub_->publish(std::move(twist_stamped_msg));
}
}  // namespace off_highway_sensor_drivers_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(off_highway_sensor_drivers_examples::ExtractVelocity)
