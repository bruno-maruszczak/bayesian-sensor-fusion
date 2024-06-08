#include "imu_module.hpp"

#include "publisher_member_function.hpp"

// Imu subscriber callback, updates imu velocities, acceleration and its covariance matrices
void Minimal::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (this->debug_)
  {
    std::string debug_msg = "Received Imu data. ";
    RCLCPP_INFO(this->get_logger(), debug_msg.c_str());
  }

  // Get angular velocity
  imu_vel_ = msg.angular_velocity;
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> imu_vel_cov_(msg.velocity_covariance);
  
  // Get acceleration
  imu_accel = msg.linear_acceleration;
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> imu_accel_cov(msg.velocity_covariance);

  // We don't use orientation
}


