# CXD5602PWBIMU ROS2 Driver
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

ROS2 package for [CXD5602PWBIMU](https://developer.sony.com/ja/spresense/specifications), a Multi-IMU for Sony Spresense.

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
# Run node directly
ros2 run cxd5602pwbimu cxd5602pwbimu_node

# Or launch with default configuration
ros2 launch cxd5602pwbimu cxd5602pwbimu.launch.xml use_madgwick:=false

# Launch with Madgwick filter enabled
ros2 launch cxd5602pwbimu cxd5602pwbimu.launch.xml use_madgwick:=true
```

## License
Apache License 2.0 - see [LICENSE](https://github.com/shunsugar/cxd5602pwbimu?tab=Apache-2.0-1-ov-file) for details.
