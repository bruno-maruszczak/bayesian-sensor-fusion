#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);