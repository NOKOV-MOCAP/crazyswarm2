[![ROS 2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)

# Crazyswarm2

A ROS 2-based stack  Swarm with nokov motion capture system native support, and vrpn is alternative.for Bitcraze Crazyflie multirotor robots.

The documentation is available here: https://imrclab.github.io/crazyswarm2/.

## Quick Start

```
git clone https://github.com/NOKOV-MOCAP/crazyswarm2.git 
cd crazyswarm2/
vcs import --input ./rosinstall --recursive
git submodule sync
git submodule update --init --recursive
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch crazyflie launch.py
```
