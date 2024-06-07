cmake_minimum_required(VERSION 3.5)
project(bayesian-sensor-fusion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  /usr/include/eigen3
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  )

add_executable(talker src/pose_utils.cpp src/publisher_member_function.cpp)
ament_target_dependencies(talker tf2 tf2_geometry_msgs rclcpp std_msgs nav_msgs)

target_link_libraries(talker gtsam Eigen3::Eigen)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}/)
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
