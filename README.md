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


## Usage

```bash
ros2 run nissan_bridge_gui battery_viewer
```

```bash
ros2 topic type /battery/voltage
```

```
nissan_bridge_msgs/msg/Float64Stamped
```

```bash
ros2 topic pub -r 10 /battery/temperature sensor_msgs/msg/Temperature "{header: auto, temperature: 48.82}"
```

## Published topics (`nissan_bridge_node`)

| Topic | Type | Unit | Source |
|---|---|---|---|
| `battery/soc` | `Float64Stamped` | 0..1 | EV 0x55B `LB_SOC` / 100 |
| `battery/voltage` | `Float64Stamped` | V | EV 0x1DB `LB_Total_Voltage` |
| `battery/current` | `Float64Stamped` | A, + = discharge | EV 0x1DB `LB_Current` × `battery_current_sign` |
| `battery/temperature` | `sensor_msgs/Temperature` | °C | CAR 0x5B3 `BatteryPackTemperature` or EV 0x5C0 `HistData_Temperature_AVG` (`battery_temperature_source`) |
| `battery/soh` | `Float64Stamped` | 0..1 | CAR 0x5B3 `BatteryStateOfHealth` / 100 |
| `battery/energy_consumed` | `Float64Stamped` | Wh | ∫V·I dt since node start |
| `battery/max_load_power` | `Float64Stamped` | W | EV 0x1DC `LB_Discharge_Power_Limit` × 1000 |
| `battery/max_charge_power` | `Float64Stamped` | W | EV 0x1DC `LB_Charge_Power_Limit` × 1000 |
| `ev/powertrain/p_mech` | `Float64Stamped` | W | V·I (battery terminal power) |
| `vehicle_status` | `geometry_msgs/TwistStamped` | m/s | `linear.x`: CAR 0x284 speed × `speed_scale`, `angular.z`: CAR 0x002 steering angle × `steering_scale` |
| `vehicle/ambient_temperature` | `sensor_msgs/Temperature` | °C | CAR 0x510 `OutsideAmbientTemperature` |

The raw per-cluster topics (`ev/battery_power`, `ev/thermal`, `ev/battery_history`, `vehicle/state`) are still published.