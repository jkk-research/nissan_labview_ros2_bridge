# Nissan LabVIEW ROS 2 Bridge

ROS 2 packages for integrating LabVIEW with the Nissan Leaf.

## Dependencies

- [ROS 2](https://docs.ros.org/en/humble/index.html) (tested with [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/) and [![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-34aec5)](https://docs.ros.org/en/jazzy/))
- A ROS 2 workspace with `colcon` installed

## Install and Build

Clone the repository into the `src` directory of a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/jkk-research/nissan_labview_ros2_bridge
```

Build all packages:

```bash
cd ~/ros2_ws
```
```bash
colcon build --packages-select nissan_bridge_msgs nissan_bridge nissan_bridge_gui --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the workspace after building:

```bash
source ~/ros2_ws/install/setup.bash
```

## Packages

- `nissan_bridge_msgs`: ROS 2 message definitions for the bridge
- `nissan_bridge`: C++ bridge node
- `nissan_bridge_gui`: Python test GUI node
