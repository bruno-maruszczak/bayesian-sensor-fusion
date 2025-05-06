## Overview

The goal is to use Bayesian optimization on factor graphs (implemented in [GTSAM](https://gtsam.org/) C++ library) to localize the robot on a known map.

Factor graphs are a **declarative model for performing Bayesian inference** particularly fitted for representing sensor fusion solutions. In our case **unary factors** represent position measurements and **binary factors** represent displacement (change of position) measurements. These measurements along with their covariance (inverse precision) matrix form constraints on possible positions and the result of **inference is the most likely sequence of positions**.

The measurements extend the factor graph by inserting appropriate factors. We use three different Turtlebot3 sensors:
- Odometry (binary factor)
- LiDAR (unary factor after reconstructing pose estimate)
- IMU (binary factor - using preintegration strategy to reduce sample count)

And 2 localization libraries: AMCL and EKF.


## Compilling and running

### If building GTSAM from source
Fix linking error (might not be needed depending on GTSAM and Eigen versions)
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


