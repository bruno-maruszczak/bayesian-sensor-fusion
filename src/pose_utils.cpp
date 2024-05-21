#include "pose_utils.hpp"

// Get yaw (rotation around z-axis) from ROS2 geometry_msg/Quaternion
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}