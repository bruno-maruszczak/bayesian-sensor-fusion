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
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace gtsam;

using PoseNoiseTuple = std::tuple<gtsam::Pose2, gtsam::noiseModel::Gaussian::shared_ptr>; 

gtsam::Pose2 operator-(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2) {
    // Subtract the translation and rotation components separately
    double dx = pose1.x() - pose2.x();
    double dy = pose1.y() - pose2.y();
    double dtheta = pose1.theta() - pose2.theta();
    // Return the difference as a new Pose2 object
    return gtsam::Pose2(dx, dy, dtheta);
}

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class Minimal : public rclcpp::Node
{
public:
  Minimal()
  : Node("minimal_publisher"), graph_n_(0), debug_(false), last_odom_reading_(Pose2(0.0, 0.0, 0.0)), odom_initialized_(false), skip_amcl(true)
  {
    // Add prior knowledge
    addPrior(Pose2(-2.0, -0.5, 0.0), noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1)));

    // Start publishing, subscribtion and time
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("estimated_pose", 10);
    
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Minimal::odom_callback, this, _1));
    
    amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10, std::bind(&Minimal::amcl_callback, this, _1));
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
  
  PoseNoiseTuple getPoseFromAMCL(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_msg)
  {
    // Get pose from msg
    auto x = amcl_msg->pose.pose.position.x;
    auto y = amcl_msg->pose.pose.position.y;
    auto theta = quaternionToYaw(amcl_msg->pose.pose.orientation);

    Pose2 pose(x, y, theta);
    
    // Convert 6x6 (x, y, z, rx, ry, rz) covariance matrix to 3x3 (x, y, rz)
    auto covariance_array = amcl_msg->pose.covariance;
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrix(covariance_array.data());
    Eigen::MatrixXd covariance(3, 3);
    covariance << covariance_matrix(0, 0), covariance_matrix(0, 1), covariance_matrix(0, 5),
                 covariance_matrix(1, 0), covariance_matrix(1, 1), covariance_matrix(1, 5),
                 covariance_matrix(5, 0), covariance_matrix(5, 1), covariance_matrix(5, 5);
    
    // Specify the desired format for printing
    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "[", "]");
    std::cout << "AMCL Pose: " << x << ", " << y << ", " << theta << std::endl;
    // Print the matrix using the specified format
    std::cout << "Covariance Matrix:\n" << covariance.format(fmt) << std::endl << std::endl;

    // Create noise model from cov matrixj
    noiseModel::Gaussian::shared_ptr noise_model = noiseModel::Gaussian::Covariance(covariance);
    return std::make_tuple(pose, noise_model);

  }
    

  void addPrior(Pose2 mean, noiseModel::Diagonal::shared_ptr noise)
  { 
    graph_.add(PriorFactor<Pose2>(1,  mean, noise));
    initial_guess_.insert(1, Pose2(-2.0, -0.5, 0.0));
    RCLCPP_INFO(this->get_logger(), "Added PriorFactor: 1");
    graph_n_++;

    if (debug_)
      RCLCPP_INFO(this->get_logger(), "Graph: Set prior to intial position");
  }

  // Given current graph, calculates and publishes estimated Pose
  // PoseWithCovarianceStamped
  void publish_esimated_pose()
  {
    // Optimise graph
    LevenbergMarquardtOptimizer optimizer(graph_, initial_guess_);
    
    // Estimate positions
    Values results = optimizer.optimize();
    Marginals marginals(graph_, results);
    

    std::string log_message = "Reading factor at: " + std::to_string(graph_n_);
    RCLCPP_INFO(this->get_logger(), log_message.c_str()); 
    auto last_position = results.at<Pose2>(graph_n_);
    auto last_covariance = marginals.marginalCovariance(graph_n_);

    // Convert Pose to msg format
    auto tf_q = tf2::Quaternion();
    tf_q.setRPY(0.0, 0.0, last_position.theta());

    auto q = tf2::toMsg(tf_q);
    
    std::array<double, 36> covariance = { 
      last_covariance(0,0), last_covariance(0,1), 0.0, 0.0, 0.0, last_covariance(0,2),
      last_covariance(1,0), last_covariance(1,1), 0.0, 0.0, 0.0, last_covariance(1,2),
      0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
      0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
      0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
      last_covariance(2,0), last_covariance(2,1), 0.0, 0.0, 0.0, last_covariance(2,2)
    };
    
    
    // Prepare message
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "odom";
    message.header.stamp = this->now();

    message.pose.pose.position.x = last_position.x();
    message.pose.pose.position.y = last_position.y();
    message.pose.pose.position.z = 0.0;
    
    message.pose.pose.orientation = q;

    message.pose.covariance = covariance;
    
    // Print output    

    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "[", "]");
    std::cout << "Estimated Pose: " << last_position.x() << ", " << last_position.y() << ", " << last_position.theta() << std::endl;
    // Print the matrix using the specified format
    std::cout << "Covariance Matrix:\n" << last_covariance.format(fmt) << std::endl;
    publisher_->publish(message);
  }
  
  // Updates Graph based on current readingsi, publishes current esitmated pose
  void update_graph() {    
    initial_guess_.insert(graph_n_+1, Pose2(-2.0, -0.5, 0.0));
    auto noise = noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));

    graph_.add(PriorFactor<Pose2>(graph_n_, amcl_pose_, amcl_noise_));
   	graph_.emplace_shared<BetweenFactor<Pose2> >(graph_n_, graph_n_ + 1, odom_reading_ - last_odom_reading_, noise);
   

    std::string message = "Added prior factor at" + std::to_string(graph_n_);
    //RCLCPP_INFO(this->get_logger(), message.c_str()); 
    message = "Added between factor at" + std::to_string(graph_n_) + " " + std::to_string(graph_n_+1);
    //RCLCPP_INFO(this->get_logger(), message.c_str()); 
    graph_n_++;
    // Publish estimated pose
    this->publish_esimated_pose();
    
    // Update last odometry, for calculating change of position 
    last_odom_reading_ = odom_reading_;
  }
 
  // Odom Reading
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (this->debug_)
    {
      std::string debug_msg = "Received odometry. " + std::to_string(graph_n_ + 1);
      RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
    }
    if(!odom_initialized_){
      last_odom_reading_ = getPoseFromOdom(msg);
      odom_initialized_ = true;
    }
    odom_reading_ = getPoseFromOdom(msg);   
  }
  
  // AMCL reading
    
  
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (this->debug_)
    {
      std::string debug_msg = "Received amcl pose.";
      RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
    }
    
    if (this->skip_amcl == true)
    {
      skip_amcl = false;
      return;
    }    

    PoseNoiseTuple result = getPoseFromAMCL(msg);
    amcl_pose_ = std::get<0>(result);
    amcl_noise_ = std::get<1>(result);
    
    if (this->debug_)
    {
      std::ostringstream stream;
      stream << "AMCL covariance matrix:\n" << amcl_noise_->R().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "[", "]"));
      RCLCPP_INFO(this->get_logger(), "Pose2: (x: %f, y: %f, theta: %f)", amcl_pose_.x(), amcl_pose_.y(), amcl_pose_.theta());
      RCLCPP_INFO(this->get_logger(), stream.str().c_str());
      std::cout << std::endl;
    }
    this->update_graph();
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  
  size_t graph_n_; // Graph node count
  bool debug_;
  Pose2 odom_reading_;
  Pose2 last_odom_reading_;
  bool odom_initialized_; 
  Pose2 amcl_pose_;
  noiseModel::Gaussian::shared_ptr amcl_noise_;

  Values initial_guess_;
  NonlinearFactorGraph graph_;
  
  bool skip_amcl;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Minimal>());
  rclcpp::shutdown();
  return 0;
}
