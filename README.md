# kalman_filter
ROS implementation of kalman filter using the available package and Eigen

## Install Eigen
```
cd ~/ && mkdir Software
cd Software
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && mkdir build && cd build
cmake ..
make
sudo make install
```
### Make sure Eigen can be found in system path
```
cd /usr/local/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

## Initialize submodule kalman-cpp
```
cd ~/catkin_ws/kalman_filter/include/kalman_lib
git submodule init
git submodule update
```