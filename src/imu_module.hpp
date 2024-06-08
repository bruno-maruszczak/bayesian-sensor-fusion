#pragma once

#include <Eigen/Dense>
#include "sensor_msgs/msg/imu.hpp"

class Minimal;
void Minimal::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
 
