# Overview
The goal is to use GTSAM library (sensor fusion) to localize the robot on a known map.
We use three different Turtlebot3 sensors:
- Odometry
- LiDAR
- IMU
And 2 localization libraries: AMCL, EKF.

# Compilling and running
### If building GTSAM from source
Fix linking error
```
cd /usr/local/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Running 
1. Start the rviz bringup
```
ros2 run bayesian-sensor-fusion rviz2.launch.py
```
2. Start the nav2 map server
```
ros2 run bayesian-sensor-fusion map.launch.py
```
3. Play included rosbag
```
ros2 bag play ros2_ws/src/bayesian-sensor-fusion/rosbags/waffle
```


