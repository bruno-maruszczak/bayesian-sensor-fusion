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

#include "pose_utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

// GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace gtsam;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Minimal : public rclcpp::Node
{
public:
  Minimal()
  : Node("minimal_publisher"), count_(0), is_graph_initialised_(false), graph_n_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Minimal::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
      1500ms, std::bind(&Minimal::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    graph_.print("Graph: ");
    publisher_->publish(message);
  }
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard something'");
    auto x = msg->pose.pose.position.x;
    auto y = msg->pose.pose.position.y;
    auto theta = quaternionToYaw(msg->pose.pose.orientation);

    odom_reading_ = Pose2(x, y, theta);
    auto noise = noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));
    if(!is_graph_initialised_){
    	is_graph_initialised_ = true;
	graph_n_++;
	graph_.addPrior(1, odom_reading_, noise);
    } else {
   	graph_.emplace_shared<BetweenFactor<Pose2> >(graph_n_, graph_n_ + 1, odom_reading_, noise);
	graph_n_++;
    }
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  Pose2 odom_reading_;
  Values graph_values_;
  NonlinearFactorGraph graph_;
  bool is_graph_initialised_;
  size_t graph_n_; // Graph node count
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Minimal>());
  rclcpp::shutdown();
  return 0;
}
