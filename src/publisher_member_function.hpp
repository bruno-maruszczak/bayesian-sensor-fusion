#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace gtsam;

using PoseNoiseTuple = std::tuple<gtsam::Pose3, gtsam::noiseModel::Gaussian::shared_ptr>; 

gtsam::Pose2 operator-(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2);
gtsam::Pose3 operator-(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2);

class Minimal : public rclcpp::Node
{
public:
    Minimal();

private:
    Pose3 getPoseFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    PoseNoiseTuple getPoseFromEKF(const nav_msgs::msg::Odometry::SharedPtr ekf_msg);
    void setPrior(Pose3 mean, noiseModel::Diagonal::shared_ptr noise);
    void publish_esimated_pose(bool debug = false);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    
    size_t graph_n_;
    bool debug_;
    Pose3 odom_reading_;
    Pose3 last_odom_reading_;
    size_t last_odom_n_;
    size_t odom_counter_;
    bool odom_initialized_; 
    Pose3 ekf_pose_;
    Pose3 amcl_pose_;

    noiseModel::Gaussian::shared_ptr ekf_noise_;
    noiseModel::Gaussian::shared_ptr amcl_noise_;
    Values initial_guess_;
    NonlinearFactorGraph graph_;
    bool skip_ekf;
    bool skip_amcl;
};
