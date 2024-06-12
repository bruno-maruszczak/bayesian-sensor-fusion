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

#include "publisher_member_function.hpp"
#include "pose_utils.hpp"

#define NaN 0.1

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace gtsam;

using PoseNoiseTuple = std::tuple<gtsam::Pose3, gtsam::noiseModel::Gaussian::shared_ptr>; 

gtsam::Pose2 operator-(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2) {
    // Subtract the translation and rotation components separately
    double dx = pose1.x() - pose2.x();
    double dy = pose1.y() - pose2.y();
    double dtheta = pose1.theta() - pose2.theta();
    // Return the difference as a new Pose2 object
    return gtsam::Pose2(dx, dy, dtheta);
}

gtsam::Pose3 operator-(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2) {
    // Subtract the translation and rotation components separately
    double dx = pose2.translation().x() - pose1.translation().x();
    double dy = pose2.translation().y() - pose1.translation().y();
    //double dz = pose1.translation().z() - pose2.translation().z();
    auto dq = Rot3(traits<Quaternion>::Between(pose1.rotation().toQuaternion(), pose2.rotation().toQuaternion()));
    // Return the difference as a new Pose2 object
    return gtsam::Pose3::Create(dq, Point3(dx, dy, 0.0));
}

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

Minimal::Minimal()
: Node("minimal_publisher"), graph_n_(0), debug_(false), last_odom_reading_(Pose3(Pose2(0.0, 0.0, 0.0))), last_odom_n_(0), odom_counter_(0), odom_initialized_(false), skip_ekf(true), skip_amcl(true)
{
  // Add prior knowledge
  setPrior(Pose3(Pose2(-2.0, -0.5, 0.0)), noiseModel::Isotropic::Sigma(6, 1e-2));

  // Start publishing, subscribtion and time
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("estimated_pose", 10);
  
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&Minimal::odom_callback, this, _1));
  
  ekf_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", 10, std::bind(&Minimal::ekf_callback, this, _1));

  amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 10, std::bind(&Minimal::amcl_callback, this, _1));
}

// Given odometry message returns robot position as Pos2(x, y, theta)
Pose3 Minimal::getPoseFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto x = msg->pose.pose.position.x;
  auto y = msg->pose.pose.position.y;
  auto theta = quaternionToYaw(msg->pose.pose.orientation);

  return Pose3(Pose2(x, y, theta));
}

PoseNoiseTuple Minimal::getPoseFromEKF(const nav_msgs::msg::Odometry::SharedPtr ekf_msg)
{

  // Get pose from msg
  auto x = ekf_msg->pose.pose.position.x;
  auto y = ekf_msg->pose.pose.position.y;
  auto theta = quaternionToYaw(ekf_msg->pose.pose.orientation);

  Pose2 pose(x, y, theta);
  
  // Convert 6x6 (x, y, z, rx, ry, rz) covariance matrix to 3x3 (x, y, rz)
  auto covariance_array = ekf_msg->pose.covariance;
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrix(covariance_array.data());
  covariance_matrix(2,2) = NaN; 
  covariance_matrix(3,3) = NaN; 
  covariance_matrix(4,4) = NaN; 

  // Create noise model from cov matrixj
  noiseModel::Gaussian::shared_ptr noise_model = noiseModel::Gaussian::Covariance(covariance_matrix);
  return std::make_tuple(Pose3(pose), noise_model);

}

PoseNoiseTuple Minimal::getPoseFromAMCL(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_msg)
{
  // Get pose from msg
  auto x = amcl_msg->pose.pose.position.x;
  auto y = amcl_msg->pose.pose.position.y;
  auto theta = quaternionToYaw(amcl_msg->pose.pose.orientation);

  Pose2 pose(x, y, theta);
  
  // Convert 6x6 (x, y, z, rx, ry, rz) covariance matrix to 3x3 (x, y, rz)
  auto covariance_array = amcl_msg->pose.covariance;
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covariance_matrix(covariance_array.data());
  covariance_matrix(2,2) = NaN; 
  covariance_matrix(3,3) = NaN; 
  covariance_matrix(4,4) = NaN; 

  // Create noise model from cov matrixj
  noiseModel::Gaussian::shared_ptr noise_model = noiseModel::Gaussian::Covariance(covariance_matrix);
  return std::make_tuple(Pose3(pose), noise_model);
}

void Minimal::setPrior(Pose3 initial_pose, noiseModel::Diagonal::shared_ptr initial_uncertainty)
{ 
  graph_.add(PriorFactor<Pose3>(1,  initial_pose, initial_uncertainty));
  initial_guess_.insert(1, initial_pose);
  graph_n_ = 1;
  RCLCPP_INFO(this->get_logger(), "Graph: Set prior to intial position");
}

// Given current graph, calculates and publishes estimated Pose
// PoseWithCovarianceStamped
void Minimal::publish_esimated_pose(bool debug)
{
  // Optimise graph
  LevenbergMarquardtOptimizer optimizer(graph_, Values(initial_guess_));
  Values results = optimizer.optimize();
  Marginals marginals(graph_, results);
  

  
  auto last_pose = results.at<Pose3>(graph_n_);
  auto last_position = last_pose.translation();
  auto last_r = last_pose.rotation().toQuaternion();
  // Convert Rotation to msg format
  auto tf_q = tf2::Quaternion(last_r.x(), last_r.y(), last_r.z(), last_r.w());
  auto q = tf2::toMsg(tf_q);
  // Convert covariance to 2D for visualisation
  auto last_covariance = marginals.marginalCovariance(graph_n_);
  std::array<double, 36> covariance = { 
    last_covariance(0,0), last_covariance(0,1), 0.0, 0.0, 0.0, last_covariance(0,5),
    last_covariance(1,0), last_covariance(1,1), 0.0, 0.0, 0.0, last_covariance(1,5),
    0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
    0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
    0.0                 , 0.0                 , 0.0, 0.0, 0.0, 0.0,
    last_covariance(5,0), last_covariance(5,1), 0.0, 0.0, 0.0, last_covariance(5,5)
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
  
  publisher_->publish(message);


  if(debug) {
    std::string log_message = "Publishing estimate at graph length of " + std::to_string(graph_n_);
    RCLCPP_INFO(this->get_logger(), log_message.c_str()); 
    // Print output    
    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "[", "]");
    std::cout << "Estimated position: " << last_position.x() << ", " << last_position.y() << ", " << last_position.z() << std::endl;
    // Print the matrix using the specified format
    std::cout << "Published Covariance Matrix:\n" << last_covariance.format(fmt) << std::endl;
  }
}


// Odom Reading
void Minimal::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::cout << "o ";
  
  if(!odom_initialized_){
    std::string debug_msg = "Initialized odom at " + std::to_string(graph_n_ + 1);
    RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
    
    last_odom_reading_ = getPoseFromOdom(msg);
    last_odom_n_ = graph_n_;
    odom_initialized_ = true;
    return;
  }
  
  if (this->debug_)
  {
    std::string debug_msg = "Received odometry reading at " + std::to_string(graph_n_ + 1);
    RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
  }

  odom_counter_++;
  if(odom_counter_ % 10 != 0) {
    return;
  }

  odom_reading_ = getPoseFromOdom(msg);   
  auto odom_noise = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-9, 1e-9, 1e-9, 1e-4).finished());
  initial_guess_.insert(graph_n_+1, Pose3(odom_reading_));
  graph_.emplace_shared<BetweenFactor<Pose3> >(last_odom_n_, graph_n_ + 1, last_odom_reading_.transformPoseTo(odom_reading_), odom_noise);

  last_odom_reading_ = odom_reading_;
  last_odom_n_ = graph_n_ + 1;
  
  graph_n_++;
  this->publish_esimated_pose();
}

// ekf reading
  

void Minimal::ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::cout << "EKF" << std::endl;
  if (this->debug_)
  {
    std::string debug_msg = "Received ekf pose at " + std::to_string(graph_n_ + 1);
    RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
  }
  
  // Skips first ekf reading
  if (this->skip_ekf == true)
  {
    skip_ekf = false;
    return;
  }    

  PoseNoiseTuple result = getPoseFromEKF(msg);
  ekf_pose_ = std::get<0>(result);
  ekf_noise_ = std::get<1>(result);

  initial_guess_.insert(graph_n_+1, Pose3(ekf_pose_));
  graph_.add(PriorFactor<Pose3>(graph_n_ + 1, ekf_pose_, ekf_noise_));

  graph_n_++;
  this->publish_esimated_pose(true);
}
 
void Minimal::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << " AMCL" << std::endl;
  if (this->debug_)
  {
    std::string debug_msg = "Received amcl pose at " + std::to_string(graph_n_ + 1);
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

  initial_guess_.insert(graph_n_+1, Pose3(amcl_pose_));
  graph_.add(PriorFactor<Pose3>(graph_n_ + 1, amcl_pose_, amcl_noise_));

  graph_n_++;
  this->publish_esimated_pose(true);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Minimal>());
  rclcpp::shutdown();
  return 0;
}
