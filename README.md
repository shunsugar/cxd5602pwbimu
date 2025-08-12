# cxd5602pwbimu ROS2 Driver
ROS2 package for cxd5602pwbimu, a Multi-IMU for SONY SPRESENSE.

## How to use
### Setup
```bash
cd ~/ros2_ws/src
git clone https://github.com/shunsugar/cxd5602pwbimu.git
wstool merge cxd5602pwbimu/cxd5602pwbimu.rosinstall
wstool update
rosdep install -r -y -i --from-paths .
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run
```
ros2 run cxd5602pwbimu cxd5602pwbimu_node.cpp
```
or
```
ros2 launch cxd5602pwbimu cxd5602pwbimu.launch.xml
```

## License
