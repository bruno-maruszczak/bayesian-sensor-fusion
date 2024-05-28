# bayesian-sensor-fusion

# Compilling and running

build gtsam from source

Fix linking error
```
cd /usr/local/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```
