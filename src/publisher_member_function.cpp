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
gtsam::Pose2 operator-(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2) {
    // Subtract the translation and rotation components separately
    double dx = pose1.x() - pose2.x();
    double dy = pose1.y() - pose2.y();
    double dtheta = pose1.theta() - pose2.theta();
    // Return the difference as a new Pose2 object
    return gtsam::Pose2(dx, dy, dtheta);
}

class Minimal : public rclcpp::Node
{
public:
  Minimal()
  : Node("minimal_publisher"), graph_n_(0), debug_(true), last_odom_reading_(Pose2(0.0, 0.0, 0.0))
  {
    // Add prior knowledge
    addPrior(Pose2(0.0, 0.0, 0.0), noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1)));

    // Start publishing, subscribtion and timer
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Minimal::topic_callback, this, _1));
    /*timer_ = this->create_wall_timer(
      3000ms, std::bind(&Minimal::timer_callback, this));
      */
  }

private:
  // Given odometry message returns robot position as Pos2(x, y, theta)
  Pose2 getPoseFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto x = msg->pose.pose.position.x;
    auto y = msg->pose.pose.position.y;
    auto theta = quaternionToYaw(msg->pose.pose.orientation);

    return Pose2(x, y, theta);
  }
  
  void addPrior(Pose2 mean, noiseModel::Diagonal::shared_ptr noise)
  { 
    graph_.addPrior(1,  mean, noise);
    initial_guess_.insert(1, Pose2(0.0, 0.0, 0.0));
    graph_n_++;

    if (debug_)
      RCLCPP_INFO(this->get_logger(), "Graph: Set prior to ");
  }

  void timer_callback()
  {
    if (this->debug_)
      RCLCPP_INFO(this->get_logger(), "Graph: Starting optimizer");

    // Optimise graph
    LevenbergMarquardtOptimizer optimizer(graph_, initial_guess_);
    
    // Estimate positions
    Values positions = optimizer.optimize();
    if (this->debug_)
      RCLCPP_INFO(this->get_logger(), "Graph: Finished optimisation");
    positions.print("Positions: ");
    auto message = std_msgs::msg::String();
    message.data = "Optimised " + std::to_string(graph_n_);

    publisher_->publish(message);
  }

  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (this->debug_)
    {
      std::string debug_msg = "Received odometry. " + std::to_string(graph_n_ + 1);
      RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
    }
    odom_reading_ = getPoseFromOdom(msg); 
    
    initial_guess_.insert(graph_n_+1, Pose2(0.0, 0.0, 0.0));
    auto noise = noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));

   	graph_.emplace_shared<BetweenFactor<Pose2> >(graph_n_, graph_n_ + 1, odom_reading_ - last_odom_reading_, noise);
    graph_n_++;
    this->timer_callback();
    last_odom_reading_ = odom_reading_;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  
  Pose2 odom_reading_;
  Pose2 last_odom_reading_;
  Values initial_guess_;
  NonlinearFactorGraph graph_;
  size_t graph_n_; // Graph node count
  bool debug_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Minimal>());
  rclcpp::shutdown();
  return 0;
}
