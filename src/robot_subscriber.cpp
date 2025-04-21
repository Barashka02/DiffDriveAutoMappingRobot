// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

class RobotSubscriber : public rclcpp::Node
{
public:
  RobotSubscriber()
  : Node("robot_subscriber")
  {
    auto camera_callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(),
                  "Camera Image Timestamp (nanoseconds): %u",
                  msg->header.stamp.nanosec);
    };
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, camera_callback);

    auto joy_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
      if (!msg->buttons.empty()) {
        RCLCPP_INFO(this->get_logger(),
                    "Game Controller Button 0 Status: %d",
                    msg->buttons[0]);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Game Controller: No buttons received.");
      }
    };
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, joy_callback);

    auto gps_callback = [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(),
                  "GPS Timestamp (nanoseconds): %u, Longitude: %f",
                  msg->header.stamp.nanosec, msg->longitude);
    };
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, gps_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSubscriber>());
  rclcpp::shutdown();
  return 0;
}

